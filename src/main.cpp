#include "mainwindow.h"
#include "thpool.h"
#include <QApplication>

#define BUFFER_SIZE 5120
#define DATA_UPDATE_HZ 33
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ
#define MAX_DELAY_TIME_US 5000

// ###### THREAD DATA ######
/// \brief a mutex to avoid multi-thread access to message
pthread_mutex_t message_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief thread pool to allow dynamic task assignment
threadpool thpool_t;
///  \brief buffer to hold received data
uint8_t buffer[BUFFER_SIZE];
/// \brief main udp data receiving thread
pthread_t recv_monitor_thread;
// #########################

/// \brief data receiving thread, that waits for data packets
/// \param mscast - poinetr to multicastpp object
void* udpReceivingThread(void *mcast);

/// \brief function used by thread pool when manipulating infomartion
/// \param packet 
void processInfo(void *packet);

/// \brief mainwindow GUI pointer
MainWindow *window;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    bool official_field = true;
    // look for parameter -l (lar field) or -o (official field). on error, defaults to -o
    if(argc!=2) ROS_ERROR("Wrong parameters, defaulting to Official field. Specify -l for Lar field or -o for Official field.");
    else {
      if(!strcmp(argv[1],"-l") || !strcmp(argv[1],"-L")){
         official_field = false;   
      } else if(!strcmp(argv[1],"-o") || !strcmp(argv[1],"-O")){
         official_field = true;   
      } else {
         ROS_ERROR("Wrong parameters, defaulting to Official field. Specify -l for Lar field or -o for Official field.");
      }
    }
    
    Multicastpp rtdb;
    window = new MainWindow(official_field,&rtdb);
    window->show();

    //Setup Thread pool and receving thread
    // #########################
    thpool_t = thpool_init(6); //5 threads per agent ?
    pthread_create(&recv_monitor_thread, NULL, udpReceivingThread, &rtdb);
    a.exec();
    pthread_join(recv_monitor_thread, NULL);
    thpool_destroy(thpool_t);
    ROS_ERROR("Cleaning resources on close ...");
    rtdb.closeSocket();
    return 0;
}

/// \brief main udp receiving thread. This thread deals with datagram reception
/// and sends the received data to be dealt by a thread using threadpool
/// \param socket - pointer to socket descriptor to use
void* udpReceivingThread(void *mcast)
{
   Multicastpp *rtdb = (Multicastpp*)mcast;
   int recvlen = 0;
   while(window->isVisible()){
      bzero(buffer, BUFFER_SIZE);
      if((recvlen = rtdb->receiveData(buffer,BUFFER_SIZE)) > 0 ){
         udp_packet *relay_packet = new udp_packet;
         relay_packet->packet = new uint8_t[recvlen];
         relay_packet->packet_size = recvlen;
         memcpy(relay_packet->packet,buffer,recvlen);
         thpool_add_work(thpool_t, processInfo, relay_packet);
      }
   }


   return NULL;
}

void processInfo(void *packet)
{
    window->updateAgentInfo(packet);
}
