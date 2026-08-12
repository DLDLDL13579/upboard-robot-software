/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>
#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#define SERVPORT 3333
#define BACKLOG 10

using namespace std;
/*互斥锁*/
pthread_mutex_t mutex_sock; // 定义一把锁
MasterConfig gMasterConfig;
double buf_socket[3];
double buf_socket_data[3];
/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage()
{
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}

/*socket通讯*/
void *socket_client(void *arg)
{
  if (arg == NULL)
  {
    printf("arg is NULL\n");
  }
  else
  {
    printf("%s\n", (char *)arg);
  }

  int count = 0;
  int sockfd, client_fd;
  int recvbytes;
  socklen_t sin_size;
  // double buf_socket[2];
  struct sockaddr_in my_addr;
  struct sockaddr_in remote_addr;
  sockfd = socket(AF_INET, SOCK_STREAM, 0); // 建立socket --
                                            // 设置端口复用
  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(int));
  my_addr.sin_family = AF_INET;         // AF_INET地址族
  my_addr.sin_port = htons(SERVPORT);   // 设定端口号(host -> networks)
  my_addr.sin_addr.s_addr = INADDR_ANY; // 32位IPv4地址
  bzero(&(my_addr.sin_zero), 8);        // 置前8个字节为0
  if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
  {
    perror("bind 出错！");
    exit(1);
  }
  if (listen(sockfd, BACKLOG) == -1) // 监听socket连接，设置队列中最多拥有连接个数为10  --
  {
    perror("listen 出错！");
    exit(1);
  }
  while (1)
  {
    sin_size = sizeof(struct sockaddr_in);                                              // 记录sockaddr_in结构体所占字节数
    if ((client_fd = accept(sockfd, (struct sockaddr *)&remote_addr, &sin_size)) == -1) // accept()缺省是阻塞函数，阻塞到有连接请求为止 --
    {
      perror("accept error");
      continue;
    }
    printf("收到一个连接来自： %s\n", inet_ntoa(remote_addr.sin_addr));
//    if (!fork())
    //{
      if (send(client_fd, "连接上了 \n", 26, 0) == -1) //--
        perror("send 出错！");
      // receive pieces of message from host
      while (1)
      {
        count++;
        // 互斥锁

      //  pthread_mutex_lock(&mutex_sock);
       // recvbytes = recv(client_fd, buf_socket, 3 * sizeof(double), 0);
        recvbytes = recv(client_fd, buf_socket, sizeof(buf_socket), 0);
      //  pthread_mutex_unlock(&mutex_sock);

        //buf_socket[recvbytes] = '\0';
        if (recvbytes == 0) // indicating that the client has been off
        {
          printf("connection to %s closed.\n", inet_ntoa(remote_addr.sin_addr));
          break;
        }

        pthread_mutex_lock(&mutex_sock);
        for(int k=0; k<3;k++){
            buf_socket_data[k]=buf_socket[k];
        }
        pthread_mutex_unlock(&mutex_sock);

        // printf("偏转角：%f , 速度:%f", buf_socket[1],buf_socket[2]);
        printf("recvbyte:%d\n", recvbytes);
        printf("count:%d\n", count);
        printf("biaozhi:%.3f\n", buf_socket_data[0]);
        printf("偏转角:%.3f\n", buf_socket_data[1]);
        printf("速度:%.3f\n", buf_socket_data[2]);

//        cout << "recvbyte:" << recvbytes << endl;
//        cout << "count:" << count << endl;
//        cout << "biaozhi:" << buf_socket_data[0] << endl;
//        cout << "偏转角:" << buf_socket_data[1] << endl;
//        cout << "速度:" << buf_socket_data[2] << endl<< endl;
        // memset((char*)&buf_socket,0,sizeof(buf_socket));//清空缓存
        // sleep(1);
      }

      close(client_fd);
      //exit(0);
    //}
   // close(client_fd);

    return ((void *)0);
  }
}

/*!
 * Setup and run the given robot controller
 */
int main_helper(int argc, char **argv, RobotController *ctrl)
{

  pthread_t tid;
  int ret;
  // 创建一条线程
  cout << "aaaaaaaaaaaaaaaa" << endl;
  pthread_mutex_init(&mutex_sock, NULL);
  ret = pthread_create(&tid, NULL, socket_client, NULL);
  if (ret != 0)
  {
    fprintf(stderr, "创建线程失败!\n");
    return -1;
  }
  //
  cout << "bbbbbbbbbbbbbbbbb" << endl;

  if (argc != 3 && argc != 4)
  {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argv[1][0] == '3')
  {
    gMasterConfig._robot = RobotType::CHEETAH_3;
  }
  else if (argv[1][0] == 'm')
  {
    gMasterConfig._robot = RobotType::MINI_CHEETAH;
  }
  else
  {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argv[2][0] == 's')
  {
    gMasterConfig.simulated = true;
  }
  else if (argv[2][0] == 'r')
  {
    gMasterConfig.simulated = false;
  }
  else
  {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argc == 4 && argv[3][0] == 'f')
  {
    gMasterConfig.load_from_file = true;
    printf("Load parameters from file\n");
  }
  else
  {
    gMasterConfig.load_from_file = false;
    printf("Load parameters from network\n");
  }

  printf("[Quadruped] Cheetah Software\n");
  printf("        Quadruped:  %s\n",
         gMasterConfig._robot == RobotType::MINI_CHEETAH ? "Mini Cheetah"
                                                         : "Cheetah 3");
  printf("        Driver: %s\n", gMasterConfig.simulated
                                     ? "Development Simulation Driver"
                                     : "Quadruped Driver");

  // dispatch the appropriate driver
  if (gMasterConfig.simulated)
  {
    if (argc != 3)
    {
      printUsage();
      return EXIT_FAILURE;
    }
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH)
    {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    }
    else if (gMasterConfig._robot == RobotType::CHEETAH_3)
    {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
    }
    else
    {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  }
  else
  {
#ifdef linux
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH)
    {
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    }
    else if (gMasterConfig._robot == RobotType::CHEETAH_3)
    {
      Cheetah3HardwareBridge hw(ctrl);
      hw.run();
    }
    else
    {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
#endif
  }
      pthread_mutex_destroy(&mutex_sock);
  return 0;
}
