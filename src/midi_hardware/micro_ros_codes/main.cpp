#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <midi_custom_interfaces/msg/encoder_data.h>
#include <midi_custom_interfaces/msg/motor_command.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

#define IN1 4
#define IN2 5
#define VREF 6
#define NSLEEP 7

struct MotorDriver
{
    uint8_t in1;
    uint8_t in2;
    uint8_t vref;
    uint8_t nsleep;

    MotorDriver(uint8_t in1, uint8_t in2, uint8_t vref, uint8_t nsleep)
    {
        this->in1 = in1;
        this->in2 = in2;
        this->vref = vref;
        this->nsleep = nsleep;
    }

    void setPins()
    {
            
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(vref, OUTPUT);
        pinMode(nsleep, OUTPUT);

        digitalWrite(nsleep, HIGH);
    }

    void setVel(int8_t pwm)
    {   
        int direction = HIGH;
        if(pwm < 0)
        {
            direction = LOW;
        }

        digitalWrite(in1, direction);
        analogWrite(vref, abs(pwm));
    }
};

struct Encoder
{
    uint8_t phaseA;
    uint8_t phaseB;

    Encoder(uint8_t phaseA, uint8_t phaseB)
    {
        this->phaseA = phaseA;
        this->phaseB = phaseB;
    }

    void setPins(void(*callback_func)())
    {
        attachInterrupt(phaseA, callback_func, RISING);
        pinMode(phaseB, INPUT);
    }
};

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

std_msgs__msg__Float64MultiArray encoder_data_msg;
std_msgs__msg__Float64MultiArray motor_cmd_msg;
rclc_executor_t executor;

MotorDriver leftMotorDriver(1, 2, 42, 41);
MotorDriver rightMotorDriver(14, 13, 22, 11);

Encoder leftEncoder(10, 9);
Encoder rightEncoder(8, 18);

volatile int64_t leftEncoderCount = 0;
volatile int64_t rightEncoderCount = 0;

unsigned long lastUpdateTime = 0;
const unsigned long loopPeriod = 10;

rcl_init_options_t init_options;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t encoder_info_pub;
rcl_subscription_t motor_command_sub;

void countLeftEncoder()
{
    if(digitalRead(leftEncoder.phaseB) == LOW)
    {
        leftEncoderCount = leftEncoderCount -1;
    }
    else
    {
        leftEncoderCount = leftEncoderCount + 1;
    }
}

void countRightEncoder()
{
    if(digitalRead(rightEncoder.phaseB) == LOW)
    {
        rightEncoderCount = rightEncoderCount -1;
    }
    else
    {
        rightEncoderCount = rightEncoderCount + 1;
    }
}

void motorCmdCallback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *cmd_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    motor_cmd_msg = *cmd_msg;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    set_microros_serial_transports(Serial);

    init_options = rcl_get_zero_initialized_init_options();
    allocator = rcl_get_default_allocator();
   /*  motor_cmd_msg.data.data = new double[2];
    encoder_data_msg.data.data = new double[2]; */

    motor_cmd_msg.data.capacity = 2;
    motor_cmd_msg.data.data = (double*)malloc(sizeof(double)*2);
    motor_cmd_msg.data.size = 0;

    encoder_data_msg.data.capacity = 2;
    encoder_data_msg.data.data = (double*)malloc(sizeof(double) * 2);
    encoder_data_msg.data.size = 0;

    /* static double mem[2];
    encoder_data_msg.data.data = mem; */

    encoder_data_msg.layout.dim.capacity = 2;
    encoder_data_msg.layout.dim.size = 0;
    encoder_data_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(encoder_data_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_hardware_node", "", &support);
    rclc_publisher_init_default(
        &encoder_info_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "encoder_data");

    rclc_subscription_init_default(
        &motor_command_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "motor_command");

    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &motor_command_sub,
        &motor_cmd_msg,
        &motorCmdCallback,
        ON_NEW_DATA);

    lastUpdateTime = millis();
}

void loop()
{   
    unsigned long currentTime = millis();

    if((currentTime - lastUpdateTime) >= loopPeriod)
    {

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        double leftEncoderCountF = static_cast<double>(leftEncoderCount);
        double rightEncoderCountF = static_cast<double>(rightEncoderCount);
        
        encoder_data_msg.data.data[0] = leftEncoderCountF;
        encoder_data_msg.data.size = 1;
        encoder_data_msg.data.data[1] = rightEncoderCountF;
        encoder_data_msg.data.size = 2;
        /* memcpy(&encoder_data_msg.data.data[0], &leftEncoderCountF, sizeof(leftEncoderCountF));
        memcpy(&encoder_data_msg.data.data[1], &rightEncoderCountF, sizeof(rightEncoderCountF)); */

        /* memcpy(&encoder_data_msg.data.data[0], &leftEncoderCountF, sizeof(leftEncoderCountF));
        memcpy(&encoder_data_msg.data.data[1], &rightEncoderCountF, sizeof(rightEncoderCountF)); */
        rcl_publish(&encoder_info_pub, &encoder_data_msg, NULL);

        int leftDir, rightDir = LOW;
        
        leftMotorDriver.setVel(motor_cmd_msg.data.data[0]);
        rightMotorDriver.setVel(motor_cmd_msg.data.data[1]);
        encoder_data_msg.data.size = 0;
    }

}