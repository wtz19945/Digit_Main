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
  W = 119,
  S = 115,
  A = 97,
  D = 100,
};

void InputListener::keyInputCallback(const std_msgs::Int32::ConstPtr& msg) {
  TkKeys key = (TkKeys) msg->data;
  //ROS_INFO("I heard: [%i]", msg->data);
  //==============Keyboard input=========//
  switch (key) {
    case TkKeys::up:
        *key_mode = 0;
        std::cout << "increase height: " << std::endl;
        break;
    case TkKeys::down:
        *key_mode = 1;
        std::cout << "decrease height: " << std::endl;
        break;
    case TkKeys::left:
        *key_mode = 2;
        std::cout << "moving left: " << std::endl;
        break;
    case TkKeys::right: //right arrow
        *key_mode = 3;
        std::cout << "moving right: " << std::endl;
        break;
    case TkKeys::space:
        *key_mode = 4;
        std::cout << "start walking: " << std::endl;
        break;
    case TkKeys::W:
        *key_mode = 5;
        std::cout << "walking forward: " << std::endl;
        break;
    case TkKeys::S:
        *key_mode = 6;
        std::cout << "walking backward: " << std::endl;
        break;
    case TkKeys::A:
        *key_mode = 7;
        std::cout << "walking left: " << std::endl;
        break;
    case TkKeys::D:
        *key_mode = 8;
        std::cout << "walking right: " << std::endl;
        break;        
    default:
        *key_mode = -1;
        break;
  }
 
}
