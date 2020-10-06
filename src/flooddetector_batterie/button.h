#ifndef BUTTON_H
#define BUTTON_H

#define button_int_Pin 2

#include <stdint.h>

namespace button {
class Button {

  public:

    Button();
    static void pressed();
    void init();
    bool isPressed();
    void reset();

  private:
  
    static bool _isPressed;
};
}
#endif
