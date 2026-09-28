#include "system_globals.h"
#include "board.h"
#include "msg/imu_msg.h"

void imu_task(void *p) {
    ImuMsg msg;

    while (1) {
        imu.Update();
        imu.GetQuaternion(msg.qw, msg.qx, msg.qy, msg.qz);
        msg.roll = imu.GetRoll();
        msg.pitch = imu.GetPitch();
        msg.yaw = imu.GetYaw();
        msg.acc_x = imu.GetAccX();
        msg.acc_y = imu.GetAccY();
        msg.acc_z = imu.GetAccZ();
        msg.gyro_x = imu.GetGyroX();
        msg.gyro_y = imu.GetGyroY();
        msg.gyro_z = imu.GetGyroZ();

        xQueueOverwrite(q_imu_state, &msg);

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}