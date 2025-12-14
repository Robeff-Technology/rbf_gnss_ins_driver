#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdexcept>

class SerialPortException : public std::runtime_error
{
public:
  SerialPortException(const std::string & message);
};

class SerialPort
{
public:
  SerialPort();
  SerialPort(const char * port_name);
  ~SerialPort();

  void open();
  void configure(unsigned int baud_rate, int data_bits = 8, char parity = 'N', int stop_bits = 1);
  void write(const char * data, int length);
  int read(char * buffer, int buffer_size);
  void close();
  void set_port_name(const char * port_name);

private:
  const char * port_name_;
  int fd_;
  bool is_open_();
  bool is_configured_;
  unsigned int baud_rate_;
  int data_bits_;
  char parity_;
  int stop_bits_;
  int buffer_size_;
  char * buffer_;
};

#endif  // SERIAL_PORT_H
