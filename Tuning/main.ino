#include <ros.h>
#include <Wire.h>
#include <math.h>
#include <Encoder.h>
#include <Adafruit_MCP4725.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <PID_v1_bc.h>

// Konstanta odometri
const float wheel_diameter = 0.36;
const float pulses_per_revolution = 400.0;
const float calibration_factor = 0.258962914;
const float koreksi_stir = 0.0628751;
const float max_steering_angle = 22.0;
const float wheel_base = 1.02;
const float track_width = 0.76;

// Pin encoder
const int encoderPin1A = 2;
const int encoderPin1B = 3;
const int encoderPin2A = 5;
const int encoderPin2B = 4;

// Hardware Timer untuk kontrol presisi
IntervalTimer controlTimer;
IntervalTimer sensorTimer;
IntervalTimer safetyTimer;

// Volatile variables untuk interrupt
volatile bool control_flag = false;
volatile bool sensor_flag = false;
volatile bool safety_flag = false;
volatile bool emergency_stop = false;

// Variabel posisi encoder
volatile long oldPosition1 = 0;
volatile long oldPosition2 = 0;
volatile long newPosition1 = 0;
volatile long newPosition2 = 0;

// PID variables - tuning yang lebih baik
float Kp = 0.6, Ki = 0.8, Kd = 0.112;
volatile float integral = 0.0;
volatile float previous_error = 0.0;
volatile float heading_error = 0.0;

// Objek encoder dan DAC
Encoder myEnc1(encoderPin1A, encoderPin1B);
Encoder myEnc2(encoderPin2A, encoderPin2B);
Adafruit_MCP4725 dac;
const float faktor_skala = 3.3 / 4095.0;

// Control variables
volatile double targetSpeed = 0.0;
volatile float currentVoltage = 0.0;
volatile float yaw_displacement = 0.0;
volatile bool acc_enabled = false;
volatile float obstacle_distance = 100.0;

// Timing variables
volatile unsigned long lastEncoderUpdate = 0;
volatile unsigned long lastValidReading = 0;
const unsigned long timeout = 3000;
const unsigned long safety_timeout = 5000;
const unsigned long lidar_timeout = 10000;

// Speed control parameters
const float voltageIncrement = 0.03;
const float maxVoltage = 3.3;
const float minVoltage = 0.0;
const float deadband = 0.1;

// ROS setup
ros::NodeHandle nh;
std_msgs::Float32 linear_displacement_msg;
std_msgs::Float32 velocity_msg;
std_msgs::Float32 sudut_msg;
ros::Publisher displacement_pub("linear_displacement", &linear_displacement_msg);
ros::Publisher velocity_pub("velocity", &velocity_msg);
ros::Publisher pub_yaw("yaw_imuz", &sudut_msg);

// Global variables for calculations
float velocity = 0.0;
float total_distance = 0.0;
unsigned long previousTime = 0;
unsigned long lastLidarUpdate = 0;

// Interrupt Service Routines
void controlTimerISR() {
    control_flag = true;
}

void sensorTimerISR() {
    sensor_flag = true;
}

void safetyTimerISR() {
    safety_flag = true;
}

// ROS Callbacks
void yawCallback(const std_msgs::Float32& msg) {
    noInterrupts();
    yaw_displacement = constrain(msg.data, -max_steering_angle, max_steering_angle);
    interrupts();
}
ros::Subscriber<std_msgs::Float32> yaw_sub("/yaw_displacement", &yawCallback);

void headingCallback(const std_msgs::Float32& msg) {
    noInterrupts();
    heading_error = msg.data;
    interrupts();
}
ros::Subscriber<std_msgs::Float32> heading_sub("/heading_error", &headingCallback);

void enableCallback(const std_msgs::Bool& msg) {
    noInterrupts();
    acc_enabled = msg.data;
    if (!acc_enabled) {
        targetSpeed = 0.0;
        currentVoltage = 0.0;
        integral = 0.0;
        previous_error = 0.0;
        emergency_stop = false;
    }
    interrupts();
    
    char log_msg[50];
    snprintf(log_msg, sizeof(log_msg), "ACC system %s", acc_enabled ? "enabled" : "disabled");
    nh.loginfo(log_msg);
}
ros::Subscriber<std_msgs::Bool> enable_sub("/auto_enable", &enableCallback);

void obstacleCallback(const std_msgs::Float32& msg) {
    noInterrupts();
    obstacle_distance = msg.data;
    lastValidReading = millis();
    lastLidarUpdate = millis();
    interrupts();
}
ros::Subscriber<std_msgs::Float32> obstacle_sub("/obstacle_distance", &obstacleCallback);

void sudutCallback(const std_msgs::Float32& msg) {
    float sudut = -(msg.data) + 14;
    if (sudut > 180) {
        sudut -= 360;
    } else if (sudut < -180) {
        sudut += 360;
    }
    sudut_msg.data = sudut;
    pub_yaw.publish(&sudut_msg);
}
ros::Subscriber<std_msgs::Float32> sub_yaw("/0dataz", sudutCallback);

void linearCallback(const std_msgs::Float32& received_msg) {
    if (acc_enabled) {
        noInterrupts();
        float newTarget = received_msg.data;
        
        // Validasi input
        if (newTarget >= 0.0 && newTarget <= maxVoltage) {
            targetSpeed = newTarget;
        }
        
        // Reset emergency jika ada command baru yang valid
        if (targetSpeed > 0.0) {
            emergency_stop = false;
        }
        
        interrupts();
        
        char log_msg[60];
        snprintf(log_msg, sizeof(log_msg), "ACC: Target speed %.2f V, Emergency: %d", 
                targetSpeed, emergency_stop);
        nh.loginfo(log_msg);
    }
}
ros::Subscriber<std_msgs::Float32> sub("/linear", &linearCallback);

// Function to update encoder readings
void updateEncoders() {
    newPosition1 = myEnc1.read();
    newPosition2 = myEnc2.read();
    
    if (newPosition1 != oldPosition1 || newPosition2 != oldPosition2) {
        lastEncoderUpdate = millis();
    }
}

// Function to calculate odometry
void calculateOdometry() {
    float distance1 = -((float)(newPosition1 - oldPosition1) / pulses_per_revolution) * (PI * wheel_diameter) * calibration_factor;
    float distance2 = -((float)(newPosition2 - oldPosition2) / pulses_per_revolution) * (PI * wheel_diameter) * calibration_factor;
    float delta_s = (distance1 + distance2) / 2.0;
    
    // Update old positions
    oldPosition1 = newPosition1;
    oldPosition2 = newPosition2;
    
    // Apply Ackermann steering correction
    if (abs(yaw_displacement) > 0.5 && abs(yaw_displacement) <= max_steering_angle) {
        float inner_wheel_radius = wheel_base / tan(abs(yaw_displacement) * PI / 180.0);
        float outer_wheel_radius = inner_wheel_radius + track_width;
        float avg_radius = (inner_wheel_radius + outer_wheel_radius) / 2.0;
        float correction_factor = 1.0 - (koreksi_stir * abs(yaw_displacement) / max_steering_angle);
        delta_s *= (avg_radius / inner_wheel_radius) * correction_factor * 0.8574867;
    }
    
    // Apply noise threshold
    if (abs(delta_s) > 0.0005) {
        total_distance += delta_s;
    }
    
    // Calculate velocity
    unsigned long currentTime = millis();
    float delta_t = (currentTime - previousTime) / 1000.0;
    if (delta_t > 0) {
        velocity = abs(delta_s / delta_t);
    }
    previousTime = currentTime;
}

// Modified PID control function with NEW SPEED ZONES
void performPIDControl() {
    if (!acc_enabled) {
        currentVoltage = 0.0;
        dac.setVoltage(0, false);
        return;
    }
    
    // Safety checks
    unsigned long currentTime = millis();
    
    // LiDAR timeout handling
    if (currentTime - lastLidarUpdate > lidar_timeout) {
        if (targetSpeed > 1.0) {
            targetSpeed *= 0.5;
        }
        char log_msg[50];
        snprintf(log_msg, sizeof(log_msg), "WARNING: LiDAR timeout, reducing speed");
        nh.logwarn(log_msg);
    }
    
    // Encoder timeout - emergency stop
    if (currentTime - lastEncoderUpdate > timeout) {
        emergency_stop = true;
        char log_msg[50];
        snprintf(log_msg, sizeof(log_msg), "EMERGENCY: Encoder timeout!");
        nh.logwarn(log_msg);
    }
    
    // Speed adjustment berdasarkan kondisi
    float adjusted_target = targetSpeed;
    
    // Emergency stop override
    if (emergency_stop) {
        adjusted_target = 0.5;
    }
    
    // Speed reduction untuk belokan tajam
    if (abs(heading_error) > 3.0) {
        float reduction_factor = 0.7;
        if (abs(heading_error) > 8.0) {
            reduction_factor = 0.5;
        }
        adjusted_target *= reduction_factor;
    }
    
    // NEW OBSTACLE-BASED SPEED CONTROL - sesuai permintaan
    if (obstacle_distance < 2.0) {
        // ZONE 1: < 2m = EMERGENCY STOP (0 km/h)
        adjusted_target = 0.0;
        char log_msg[60];
        snprintf(log_msg, sizeof(log_msg), "EMERGENCY STOP: Obstacle at %.1fm (< 2.0m)", obstacle_distance);
        nh.logwarn(log_msg);
    } else if (obstacle_distance < 4.0) {
        // ZONE 2: 2-4m = LOW SPEED (setara 4 km/h)
        float target_4kmh_voltage = (4.0 / 15.0) * 3.3; // ~0.88V
        if (adjusted_target > target_4kmh_voltage) {
            adjusted_target = target_4kmh_voltage;
        }
        char log_msg[60];
        snprintf(log_msg, sizeof(log_msg), "LOW SPEED: Obstacle at %.1fm (2-4m), max 4km/h", obstacle_distance);
        nh.loginfo(log_msg);
    } else if (obstacle_distance < 6.0) {
        // ZONE 3: 4-6m = MEDIUM SPEED (setara 8 km/h)
        float target_8kmh_voltage = (8.0 / 15.0) * 3.3; // ~1.76V
        if (adjusted_target > target_8kmh_voltage) {
            adjusted_target = target_8kmh_voltage;
        }
        char log_msg[60];
        snprintf(log_msg, sizeof(log_msg), "MEDIUM SPEED: Obstacle at %.1fm (4-6m), max 8km/h", obstacle_distance);
        nh.loginfo(log_msg);
    }
    // ZONE 4: > 6m = HIGH SPEED (setara 12 km/h) - tidak ada pembatasan tambahan
    
    // PID calculation
    float error = adjusted_target - velocity;
    
    // Anti-windup integration
    if (abs(error) < 2.0) {
        integral += error * 0.1;
        integral = constrain(integral, -5.0, 5.0);
    }
    
    float derivative = (error - previous_error) / 0.1;
    float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previous_error = error;
    
    // Smooth voltage control
    float target_voltage = constrain(pid_output, minVoltage, maxVoltage);
    
    // Deadband untuk stabilitas
    if (abs(target_voltage - currentVoltage) < deadband && abs(error) < 0.1) {
        target_voltage = currentVoltage;
    }
    
    if (adjusted_target > 0.0) {
        // Smooth acceleration/deceleration
        if (currentVoltage < target_voltage) {
            currentVoltage += voltageIncrement;
            if (currentVoltage > target_voltage) {
                currentVoltage = target_voltage;
            }
        } else if (currentVoltage > target_voltage) {
            currentVoltage -= voltageIncrement;
            if (currentVoltage < target_voltage) {
                currentVoltage = target_voltage;
            }
        }
    } else {
        // Emergency braking - lebih cepat untuk zona < 2m
        currentVoltage -= 3 * voltageIncrement; // Lebih cepat dari sebelumnya
        if (currentVoltage < 0) currentVoltage = 0;
    }
    
    // Apply voltage
    dac.setVoltage((int)(currentVoltage / faktor_skala), false);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize ROS
    nh.initNode();
    nh.advertise(displacement_pub);
    nh.advertise(velocity_pub);
    nh.advertise(pub_yaw);
    
    // Subscribe to topics
    nh.subscribe(heading_sub);
    nh.subscribe(yaw_sub);
    nh.subscribe(sub);
    nh.subscribe(enable_sub);
    nh.subscribe(obstacle_sub);
    nh.subscribe(sub_yaw);
    
    // Initialize DAC
    dac.begin(0x60);
    dac.setVoltage(0, false);
    
    // Wait for ROS connection
    while (!nh.connected()) {
        nh.spinOnce();
        delay(100);
    }
    
    // Initialize hardware timers
    controlTimer.begin(controlTimerISR, 100000);    // 10 Hz control loop
    sensorTimer.begin(sensorTimerISR, 50000);       // 20 Hz sensor reading
    safetyTimer.begin(safetyTimerISR, 500000);      // 2 Hz safety check
    
    nh.loginfo("Teensy ACC system initialized with NEW SPEED ZONES:");
    nh.loginfo("< 2m = STOP, 2-4m = 4km/h, 4-6m = 8km/h, > 6m = 12km/h");
    
    // Initialize timing variables
    lastEncoderUpdate = millis();
    lastValidReading = millis();
    lastLidarUpdate = millis();
    previousTime = millis();
}

void loop() {
    // Handle sensor readings at 20Hz
    if (sensor_flag) {
        sensor_flag = false;
        updateEncoders();
        calculateOdometry();
    }
    
    // Handle control at 10Hz
    if (control_flag) {
        control_flag = false;
        performPIDControl();
        
        // Publish telemetry
        linear_displacement_msg.data = total_distance;
        displacement_pub.publish(&linear_displacement_msg);
        velocity_msg.data = velocity;
        velocity_pub.publish(&velocity_msg);
    }
    
    // Handle safety checks at 2Hz
    if (safety_flag) {
        safety_flag = false;
        
        unsigned long currentTime = millis();
        
        // Reset emergency jika semua sistem normal
        if ((currentTime - lastEncoderUpdate < timeout) && acc_enabled) {
            emergency_stop = false;
        }
        
        // Debug output dengan info zona baru
        static int debug_counter = 0;
        if (++debug_counter >= 2) {
            debug_counter = 0;
            char log_msg[120];
            char zone_info[20];
            
            // Tentukan zona berdasarkan obstacle distance
            if (obstacle_distance < 2.0) {
                strcpy(zone_info, "STOP");
            } else if (obstacle_distance < 4.0) {
                strcpy(zone_info, "4km/h");
            } else if (obstacle_distance < 6.0) {
                strcpy(zone_info, "8km/h");
            } else {
                strcpy(zone_info, "12km/h");
            }
            
            snprintf(log_msg, sizeof(log_msg), 
                    "V=%.2f T=%.2f CV=%.2f Obs=%.1fm Zone=%s E=%d", 
                    velocity, targetSpeed, currentVoltage, obstacle_distance, 
                    zone_info, emergency_stop);
            nh.loginfo(log_msg);
        }
    }
    
    // Handle ROS communication
    nh.spinOnce();
    
    // Small delay
    delayMicroseconds(50);
}