#include "input_listener.hpp"
#include "utilities.hpp"

InputListener::InputListener(int *key_mode) : 
    key_mode(key_mode)
{
  key_sub = node_handler.subscribe("/keyinput", 10, &InputListener::keyInputCallback, this);
}

//Source: https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/key-names.html
enum class TkKeys {
  left = 65361,
  up = 65362,
  right = 65363,
  down = 65364,
  space = 32,
};

void InputListener::keyInputCallback(const std_msgs::Int32::ConstPtr& msg) {
  TkKeys key = (TkKeys) msg->data;
  // ROS_INFO("I heard: [%i]", msg->data);
  //==============Keyboard input=========//
  switch (key) {
    case TkKeys::up:
        *key_mode = 0;
        std::cout << "increasing standing height: " << std::endl;
        break;
    case TkKeys::down:
        *key_mode = 1;
        std::cout << "decreasing standing height: " << std::endl;
        break;
    case TkKeys::left:
        *key_mode = 2;
        std::cout << "moving left: " << std::endl;
        break;
    case TkKeys::right: //right arrow
        *key_mode = 3;
        std::cout << "moving right: " << std::endl;
        break;
    default:
        *key_mode = -1;
        break;
  }
 
}
