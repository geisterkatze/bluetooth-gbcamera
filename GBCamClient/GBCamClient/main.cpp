//
//  main.cpp
//  GBCamClient
//
//  Created by goatspit on 18/03/2017.
//  Copyright © 2017 goatspit. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>


int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tcgetattr\n", errno);
        return -1;
    }
    
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr\n", errno);
        return -1;
    }
    return 0;
}

void
set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tggetattr\n", errno);
        return;
    }
    
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes\n", errno);
}


void writeTGAHeader(int fd) {
    uint8_t header[] = {
        0,
        0,
        3,    // Grayscale
        0, 0,
        0, 0,
        0,
        0, 0, // X origin
        0, 0, // y origin
        128,
        0,
        123,
        0,
        8,    // 8 bit bitmap
        0x30,
    };
    write(fd, &header, sizeof(header));
}


int main(int argc, const char * argv[]) {
    time_t currentTime = time(NULL);
    struct tm *localTime = localtime(&currentTime);
    char fileName[1000];
    char timeString[1000];
    if (!strftime(timeString, sizeof(timeString), "%F_%H-%M-%S", localTime)) {
        printf ("Could not generate date string: %s", strerror (errno));
        return 1;
    }
    snprintf(fileName, sizeof(fileName), "./gcbam-%s.tga", timeString);
    int imageFileDescriptor = open(fileName, O_WRONLY | O_TRUNC | O_CREAT);
    if (imageFileDescriptor < 0)
    {
        printf ("error %d opening %s: %s\n", errno, fileName, strerror (errno));
        return 1;
    }
    
    const char *portname = "/dev/cu.usbmodem1411";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf ("error %d opening %s: %s\n", errno, portname, strerror (errno));
        return 1;
    }
    
    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 1); // set blocking
    
    write (fd, "T", 1);
    
    usleep (25 * 100); // sleep enough to transmit the character

    // receive 25:  approx 100 uS per char transmit
    char buf [1000];
    size_t transmissionLength = 0;
    writeTGAHeader(imageFileDescriptor);
    while(const size_t n = read (fd, buf, sizeof buf)) { // read up to 100 characters if ready to read
        write(imageFileDescriptor, buf, n);
        transmissionLength += n;
        if (transmissionLength >= 128 * 123) {
            break;
        }
    };

    close(imageFileDescriptor);
    close(fd);
    printf("Serial port closed.\n");
    
    return 0;
}
