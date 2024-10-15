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
  I = 105,
  K = 107,
  J = 106,
  L = 108,
  Z = 122,
  picth_up = 65464,
  pitch_down = 65461,
  roll_up = 65460,
  roll_down = 65462,
  soft_down = 65421
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
        std::cout << "turning clockwise: " << std::endl;
        break;
    case TkKeys::right: //right arrow
        *key_mode = 3;
        std::cout << "turning counter-clockwise: " << std::endl;
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

    case TkKeys::I:
        *key_mode = 9;
        std::cout << "drift forward: " << std::endl;
        break;        

    case TkKeys::K:
        *key_mode = 10;
        std::cout << "drift backward: " << std::endl;
        break;        

    case TkKeys::J:
        *key_mode = 11;
        std::cout << "drift left: " << std::endl;
        break;        

    case TkKeys::L:
        *key_mode = 12;
        std::cout << "drift right: " << std::endl;
        break;

    case TkKeys::Z:
        *key_mode = 13;
        break;

    case TkKeys::picth_up:
        *key_mode = 14;
        std::cout << "pitch up: " << std::endl;
        break;

    case TkKeys::pitch_down:
        *key_mode = 15;
        std::cout << "pitch down: " << std::endl;
        break;

    case TkKeys::roll_up:
        *key_mode = 16;
        std::cout << "roll up: " << std::endl;
        break;

    case TkKeys::roll_down:
        *key_mode = 17;
        std::cout << "roll down: " << std::endl;
        break;
    
    case TkKeys::soft_down:
        *key_mode = 18;
        std::cout << "turn off: " << std::endl;
        break;

    default:
        *key_mode = -1;
        break;
  }
 
}
