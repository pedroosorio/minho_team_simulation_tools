#include "multicastpp.h"

#define PERRNO(txt) \
    printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define PERR(txt, par...) \
    printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)

#ifdef DEBUG
#define PDEBUG(txt, par...) \
    printf("DEBUG: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#else
#define PDEBUG(txt, par...)
#endif

Multicastpp::Multicastpp()
{
    socket_fd = openSocket("wlan0",1);
}

int if_NameToIndex(std::string ifname, char *address)
{
    int	fd;
    struct ifreq if_info;
    int if_index;

    memset(&if_info, 0, sizeof(if_info));
    strncpy(if_info.ifr_name, ifname.c_str(), IFNAMSIZ-1);

    if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        ROS_ERROR("Error getting UDP socket");
        return -1;
    }
    if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
    {
        ROS_ERROR("Error sending IOCTL 1. Wrong interface name.");
        close(fd);
        return -1;
    }
    if_index = if_info.ifr_ifindex;

    if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
    {
        ROS_ERROR("Error sending IOCTL 2");
        close(fd);
        return -1;
    }

    close(fd);

    sprintf(address, "%d.%d.%d.%d",
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[2],
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[3],
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[4],
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[5]);

    ROS_INFO("Using device %s : Agent IP -> %s", if_info.ifr_name, address);

    return if_index;
}


//	*************************
//  Open Socket
//
int Multicastpp::openSocket(std::string interface, int recv_own_data)
{
   struct sockaddr_in multicastAddress;
   struct ip_mreqn mreqn;
   struct ip_mreq mreq;
    int multiSocket;
    int opt;
    char address[20];

   bzero(&multicastAddress, sizeof(struct sockaddr_in));
   multicastAddress.sin_family = AF_INET;
   multicastAddress.sin_port = htons(MULTICAST_PORT);
   multicastAddress.sin_addr.s_addr = INADDR_ANY;

    bzero(&destAddress, sizeof(struct sockaddr_in));
    destAddress.sin_family = AF_INET;
    destAddress.sin_port = htons(MULTICAST_PORT);
    destAddress.sin_addr.s_addr = inet_addr(MULTICAST_IP);

    if((multiSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        ROS_ERROR("Error creating Multicast UDP socket : %s",strerror(errno));
        return -1;
    }

    memset((void *) &mreqn, 0, sizeof(mreqn));
    mreqn.imr_ifindex=if_NameToIndex(interface, address);

    ip_base.assign(address,20);
    std::string temp = ip_base.substr(ip_base.find_last_of(".")+1,ip_base.size());
    ip_base = ip_base.substr(0,ip_base.find_last_of("."));
    if(strcmp("172.16.49",ip_base.c_str())!=0) agent_id = 0;
    else agent_id = std::stoi(temp);

    printf("Ip Base: %s | Agent ID: %d",ip_base.c_str(),agent_id);
    if((setsockopt(multiSocket, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
    {
        ROS_ERROR("Error setting socket option 1: %s",strerror(errno));
        return -1;
    }

    opt = 1;
    if((setsockopt(multiSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
    {
        ROS_ERROR("Error setting socket option 2: %s",strerror(errno));
        return -1;
    }

    memset((void *) &mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_IP);
    mreq.imr_interface.s_addr = inet_addr(address);

    if((setsockopt(multiSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
    {
        ROS_ERROR("Error setting socket option 3: %s",strerror(errno));
        return -1;
    }

    /* Disable reception of our own multicast */
    opt = recv_own_data;
    if((setsockopt(multiSocket, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
    {
        ROS_ERROR("Error setting socket option 4: %s",strerror(errno));
        return -1;
    }

    if(bind(multiSocket, (struct sockaddr *) &multicastAddress, sizeof(struct sockaddr_in)) == -1)
    {
        ROS_ERROR("Error binding socket: %s",strerror(errno));
        return -1;
    }

    return (multiSocket);
}



//	*************************
//  Close Socket
//
void Multicastpp::closeSocket()
{
    if(socket_fd != -1)
        shutdown(socket_fd, SHUT_RDWR);
}



//	*************************
//  Send Data
//
int Multicastpp::sendData(void* data, int dataSize)
{
    return sendto(socket_fd, data, dataSize, 0, (struct sockaddr *)&destAddress, sizeof (struct sockaddr));
}



//	*************************
//  Receive Data
//
int Multicastpp::receiveData(void* buffer, int bufferSize)
{
    return recv(socket_fd, buffer, bufferSize, 0);
}
