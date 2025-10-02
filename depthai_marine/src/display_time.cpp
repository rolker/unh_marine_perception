#include <chrono>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
  std::stringstream ss;
  ss << std::fixed;
  const auto now = std::chrono::system_clock::now();
  ss << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()/1000000.0;
  int output_size = ss.str().size()+3;
  std::string eraser(output_size, '\b');

  std::cout << std::fixed;
  std::cout << "\n\n\n\n\n          " << ss.str() << "   ";
  while(true)
  {
    const auto now = std::chrono::system_clock::now();
    std::cout << eraser << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()/1000000.0 << "   ";
    std::cout.flush();
  }
  return 0;
}
