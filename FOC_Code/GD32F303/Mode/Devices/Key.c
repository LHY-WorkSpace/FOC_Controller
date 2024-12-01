#include "Key.h"
#include "DataType.h"
#include "gd32f30x.h"

#define EVENT_CB(ev)   if(handle->cb[ev])handle->cb[ev]((Button*)handle)


static struct Button* head_handle = NULL;

static uint8_t KeyState[Key_MaxNum];

KeyInfo_t KeyInfo;

struct Button Button_Up;
struct Button Button_Down;
struct Button Button_Left;
struct Button Button_Right;

void Button_init(struct Button* handle, uint8_t(*pin_level)(uint8_t), uint8_t active_level, uint8_t button_id)
{
	memset(handle, 0, sizeof(struct Button));
	handle->event = (uint8_t)NONE_PRESS;
	handle->hal_button_Level = pin_level;
	handle->button_level = handle->hal_button_Level(button_id);
	handle->active_level = active_level;
	handle->button_id = button_id;
}


void button_attach(struct Button* handle, PressEvent event, BtnCallback cb)
{
	handle->cb[event] = cb;
}


PressEvent get_button_event(struct Button* handle)
{
	return (PressEvent)(handle->event);
}


void button_handler(struct Button* handle)
{
	uint8_t read_gpio_level = handle->hal_button_Level(handle->button_id);

	//ticks counter working..
	if((handle->state) > 0) handle->ticks++;

	/*------------button debounce handle---------------*/
	if(read_gpio_level != handle->button_level) { //not equal to prev one
		//continue read 3 times same new level change
		if(++(handle->debounce_cnt) >= DEBOUNCE_TICKS) {
			handle->button_level = read_gpio_level;
			handle->debounce_cnt = 0;
		}
	} else { //leved not change ,counter reset.
		handle->debounce_cnt = 0;
	}

	/*-----------------State machine-------------------*/
	switch (handle->state) {
	case 0:
		if(handle->button_level == handle->active_level) {	//start press down
			handle->event = (uint8_t)PRESS_DOWN;
			EVENT_CB(PRESS_DOWN);
			handle->ticks = 0;
			handle->repeat = 1;
			handle->state = 1;
		} else {
			handle->event = (uint8_t)NONE_PRESS;
		}
		break;

	case 1:
		if(handle->button_level != handle->active_level) { //released press up
			handle->event = (uint8_t)PRESS_UP;
			EVENT_CB(PRESS_UP);
			handle->ticks = 0;
			handle->state = 2;

		} else if(handle->ticks > LONG_TICKS) {
			handle->event = (uint8_t)LONG_PRESS_START;
			EVENT_CB(LONG_PRESS_START);
			handle->state = 5;
		}
		break;

	case 2:
		if(handle->button_level == handle->active_level) { //press down again
			handle->event = (uint8_t)PRESS_DOWN;
			EVENT_CB(PRESS_DOWN);
			handle->repeat++;
			EVENT_CB(PRESS_REPEAT); // repeat hit
			handle->ticks = 0;
			handle->state = 3;
		} else if(handle->ticks > SHORT_TICKS) { //released timeout
			if(handle->repeat == 1) {
				handle->event = (uint8_t)SINGLE_CLICK;
				EVENT_CB(SINGLE_CLICK);
			} else if(handle->repeat == 2) {
				handle->event = (uint8_t)DOUBLE_CLICK;
				EVENT_CB(DOUBLE_CLICK); // repeat hit
			}
			handle->state = 0;
		}
		break;

	case 3:
		if(handle->button_level != handle->active_level) { //released press up
			handle->event = (uint8_t)PRESS_UP;
			EVENT_CB(PRESS_UP);
			if(handle->ticks < SHORT_TICKS) {
				handle->ticks = 0;
				handle->state = 2; //repeat press
			} else {
				handle->state = 0;
			}
		}else if(handle->ticks > SHORT_TICKS){ // long press up
			handle->state = 0;
		}
		break;

	case 5:
		if(handle->button_level == handle->active_level) {
			//continue hold trigger
			handle->event = (uint8_t)LONG_PRESS_HOLD;
			EVENT_CB(LONG_PRESS_HOLD);

		} else { //releasd
			handle->event = (uint8_t)PRESS_UP;
			EVENT_CB(PRESS_UP);
			handle->state = 0; //reset
		}
		break;
    default:
        handle->state = 0; //reset
        break;
	}
}


int button_start(struct Button* handle)
{
	struct Button* target = head_handle;
	while(target) {
		if(target == handle) return -1;	//already exist.
		target = target->next;
	}
	handle->next = head_handle;
	head_handle = handle;
	return 0;
}


void button_stop(struct Button* handle)
{
	struct Button** curr;
	for(curr = &head_handle; *curr; ) {
		struct Button* entry = *curr;
		if (entry == handle) {
			*curr = entry->next;
			return;//glacier add 2021-8-18
		} else
			curr = &entry->next;
	}
}


void button_ticks()
{
	struct Button* target;
	for(target=head_handle; target; target=target->next) 
    {
		button_handler(target);
	}
}



uint8_t read_button_GPIO(uint8_t button_id)
{
	// you can share the GPIO read function with multiple Buttons
	switch(button_id)
	{
		case Key_Up:
			return KEY_UP;
		// case Key_Down:
		// 	return KEY_DOWN;
		// case Key_Left:
		// 	return KEY_LEFT;
		// case Key_Right:
		// 	return KEY_RIGHT;
		default:
			return 0;
	}
}

void PRESS_DOWN_Handler(void *Data)
{
    struct Button *Tmp;

    Tmp = (struct Button *)Data;

    switch (Tmp->button_id)
    {
        case Key_Up:
			KeyState[Key_Up] = PRESS_DOWN;
            break;
        default:
            break;
    }
}


// void PRESS_UP_Handler(void *Data)
// {
//     struct Button *Tmp;
//     Tmp = (struct Button *)Data;
//     switch (Tmp->button_id)
//     {
//         case Key_Up:
//             break;
//         case Key_Down:
//             break;
//         case Key_Left:
//             break;
//         case Key_Right:
//             break;
//         default:
//             break;
//     }
// }

// void PRESS_REPEAT_Handler(void *Data)
// {
//     struct Button *Tmp;
//     Tmp = (struct Button *)Data;
//     switch (Tmp->button_id)
//     {
//         case Key_Up:
//             break;
//         case Key_Down:
//             break;
//         case Key_Left:
//             break;
//         case Key_Right:
//             break;
//         default:
//             break;
//     }
// }



void SINGLE_Click_Handler(void *Data)
{
    struct Button *Tmp;
    Tmp = (struct Button *)Data;
    switch (Tmp->button_id)
    {
        case Key_Up:
			KeyState[Key_Up] = SINGLE_CLICK;
            break;
        default:
            break;
    }
}

void DOUBLE_Click_Handler(void *Data)
{
    struct Button *Tmp;
    Tmp = (struct Button *)Data;
    switch (Tmp->button_id)
    {
        case Key_Up:
			KeyState[Key_Up] = DOUBLE_CLICK;
            break;
        default:
            break;
    }
}

void LONG_PRESS_START_Handler(void *Data)
{
    struct Button *Tmp;
    Tmp = (struct Button *)Data;
    switch (Tmp->button_id)
    {
        case Key_Up:
			KeyState[Key_Up] = LONG_PRESS_START;
            break;
        default:
            break;
    }
}

void LONG_PRESS_HOLD_Handler(void *Data)
{
    struct Button *Tmp;
    Tmp = (struct Button *)Data;
    switch (Tmp->button_id)
    {
        case Key_Up:
			KeyState[Key_Up] = LONG_PRESS_HOLD;
            break;
        default:
            break;
    }
}


void Key_IO_Init()
{

	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;	
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
}




void Key_Init()
{
	Key_IO_Init();

    Button_init(&Button_Up,read_button_GPIO,RESET,Key_Up);
    // Button_init(&Button_Down,read_button_GPIO,RESET,Key_Down);
    // Button_init(&Button_Left,read_button_GPIO,RESET,Key_Left);
    // Button_init(&Button_Right,read_button_GPIO,RESET,Key_Right);


    button_attach(&Button_Up, PRESS_DOWN,       PRESS_DOWN_Handler);
	// button_attach(&Button_Up, PRESS_UP,         PRESS_UP_Handler);
	// button_attach(&Button_Up, PRESS_REPEAT,     PRESS_REPEAT_Handler);
	// button_attach(&Button_Up, SINGLE_CLICK,     SINGLE_Click_Handler);
	button_attach(&Button_Up, DOUBLE_CLICK,     DOUBLE_Click_Handler);
	button_attach(&Button_Up, LONG_PRESS_START, LONG_PRESS_START_Handler);
	// button_attach(&Button_Up, LONG_PRESS_HOLD,  LONG_PRESS_HOLD_Handler);

    // button_attach(&Button_Down, PRESS_DOWN,       PRESS_DOWN_Handler);
	// button_attach(&Button_Down, PRESS_UP,         PRESS_UP_Handler);
	// button_attach(&Button_Down, PRESS_REPEAT,     PRESS_REPEAT_Handler);
	// button_attach(&Button_Down, SINGLE_CLICK,     SINGLE_Click_Handler);
	// button_attach(&Button_Down, DOUBLE_CLICK,     DOUBLE_Click_Handler);
	// button_attach(&Button_Down, LONG_PRESS_START, LONG_PRESS_START_Handler);
	// button_attach(&Button_Down, LONG_PRESS_HOLD,  LONG_PRESS_HOLD_Handler);

    // button_attach(&Button_Left, PRESS_DOWN,       PRESS_DOWN_Handler);
	// button_attach(&Button_Left, PRESS_UP,         PRESS_UP_Handler);
	// button_attach(&Button_Left, PRESS_REPEAT,     PRESS_REPEAT_Handler);
	// button_attach(&Button_Left, SINGLE_CLICK,     SINGLE_Click_Handler);
	// button_attach(&Button_Left, DOUBLE_CLICK,     DOUBLE_Click_Handler);
	// button_attach(&Button_Left, LONG_PRESS_START, LONG_PRESS_START_Handler);
	// button_attach(&Button_Left, LONG_PRESS_HOLD,  LONG_PRESS_HOLD_Handler);


    // button_attach(&Button_Right, PRESS_DOWN,       PRESS_DOWN_Handler);
	// button_attach(&Button_Right, PRESS_UP,         PRESS_UP_Handler);
	// button_attach(&Button_Right, PRESS_REPEAT,     PRESS_REPEAT_Handler);
	// button_attach(&Button_Right, SINGLE_CLICK,     SINGLE_Click_Handler);
	// button_attach(&Button_Right, DOUBLE_CLICK,     DOUBLE_Click_Handler);
	// button_attach(&Button_Right, LONG_PRESS_START, LONG_PRESS_START_Handler);
	// button_attach(&Button_Right, LONG_PRESS_HOLD,  LONG_PRESS_HOLD_Handler);

    button_start(&Button_Up);
    // button_start(&Button_Down);
    // button_start(&Button_Left);
    // button_start(&Button_Right);
	memset(&KeyState,0xff,sizeof(KeyState));
}   



KeyInfo_t GetKeyState()
{
	uint8_t i;
	for( i = 0; i < Key_MaxNum; i++)
	{
		if(KeyState[i] != 0xff)
		{
			break; 
		}
	}

	if( i >= Key_MaxNum) 
	{
		KeyInfo.KeyState = NONE_PRESS;
		KeyInfo.KeyNum = Key_MaxNum;
	}
	else
	{
		KeyInfo.KeyNum = i;
		KeyInfo.KeyState = (PressEvent)KeyState[i];
	}

	memset(&KeyState,0xff,sizeof(KeyState));

	return KeyInfo;

}
