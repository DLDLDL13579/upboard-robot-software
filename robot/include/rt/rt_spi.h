/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */

#ifndef _rt_spi
#define _rt_spi

#ifdef linux  //todo202606新增todo  这个文件只在linux下编译，其他平台不编译------------------------------------------------------------------------------

#include <fcntl.h>      //Needed for SPI port
#include <sys/ioctl.h>  //Needed for SPI port

// incredibly obscure bug in SPI_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

#include <linux/spi/spidev.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  //Needed for SPI port
#include <spi_command_t.hpp>
#include <spi_data_t.hpp>
#include <spi_torque_t.hpp>

#define K_EXPECTED_COMMAND_SIZE 256 // 202606新增todo 这个是根据spi_command_t的实际大小来的，需测试
#define K_WORDS_PER_MESSAGE  86//66 202606新增
#define K_EXPECTED_DATA_SIZE 116 // 202606新增todo 这个是根据spi_data_t的实际大小来的，需测试
//#define K_KNEE_OFFSET_POS 4.31f   //lqh 2021.7.24 old :4.35f
#define K_KNEE_OFFSET_POS 0.4//2.7925f //0.04f//3.55f   //min high 134mm  angle 0.55  2.59/.73  cc7.04 2026新增 根据小腿电机调0来定
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'),     \
      (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), \
      (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

void init_spi();

void spi_send_receive(spi_command_t* command, spi_data_t* data);
void spi_driver_run();

spi_data_t* get_spi_data();
spi_command_t* get_spi_command();

/*!
 * SPI command message 15*2*4+1*2*4+4 = 132 bytes=66 words 
 * 132 bytes+5*2*4=172 bytes=86 words 202606新增
 */
typedef struct {
  float q_des_abad[2];
  float q_des_hip[2];
  float q_des_knee[2];
  float q_des_sup[2];//202606新增

  float qd_des_abad[2];
  float qd_des_hip[2];
  float qd_des_knee[2];
  float qd_des_sup[2];//202606新增

  float kp_abad[2];
  float kp_hip[2];
  float kp_knee[2];
  float kp_sup[2];  //202606新增

  float kd_abad[2];
  float kd_hip[2];
  float kd_knee[2];
  float kd_sup[2];  //202606新增

  float tau_abad_ff[2];
  float tau_hip_ff[2];
  float tau_knee_ff[2];
  float tau_sup_ff[2];//202606新增

  int32_t flags[2];
  int32_t checksum;

} spine_cmd_t;

/*!
 * SPI data message
 * 6*2*4+1*2*4+4=60 bytes=30 words
 * 60 bytes+3*2*4=84 bytes=42 words
 * 84 bytes+3*2*4=108 bytes=54 words 202606新增
 */
typedef struct {
  float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  float q_sup[2];//202606新增

  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  float qd_sup[2];  //202606新增

  float tau_abad[2];
  float tau_hip[2];
  float tau_knee[2];
  float tau_sup[2];//202606新增

  int32_t flags[2];
  int32_t checksum;


} spine_data_t;

#endif // END of #ifdef linux

#endif

