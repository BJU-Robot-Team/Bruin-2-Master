#include <iostream>
#include <cstdlib>

int main() {
  //system("ls");
  std::string dev_name = "tty";   // Change this accordingly for the device
  char pot_val = '\xFF';
  std::string term_cmds = "echo -n -e '\xFE\xAA\x00";
  std::string dev_acc = "' > /dev/";
  std::string foo = term_cmds + pot_val + dev_acc + dev_name + '\n';
  const char* cmd = foo.c_str();
  printf(cmd);    // To print out what is about to be sent to the system terminal

  // Writes directly to the digiPot the instruction set
  system(cmd);
  return 0;
}
