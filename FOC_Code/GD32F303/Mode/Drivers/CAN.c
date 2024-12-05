#include "gd32f30x.h"
#include <stdio.h>
#include "DataType.h"
#include "CAN.h"


can_receive_message_struct receive_message;
can_trasnmit_message_struct transmit_message;



static void CAN_GPIO_Init()
{
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOB,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_8);//CAN_RX
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);//CAN_TX
    gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP,ENABLE);//部分重映射

}



void CAN_Network_Init(void)
{
    can_parameter_struct            can_parameter;
    can_filter_parameter_struct     can_filter;
    
	CAN_GPIO_Init();

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    
    can_deinit(CANX);
    
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;

    /* baudrate 1Mbps  = 6 */
    can_parameter.prescaler = CAN_CLK;
    can_init(CANX, &can_parameter);

    // can_filter.filter_number = 0;
    can_filter.filter_number = 15;
   
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);

    /* enable CAN receive FIFO1 not empty interrupt */
    can_interrupt_enable(CANX, CAN_INT_RFNE1);
	
	nvic_irq_enable(CAN0_RX1_IRQn,2,0);

}



void CAN_Test()
{
    uint16_t timeout;
    uint8_t transmit_mailbox;


	CAN_Network_Init();

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = 0x00;
    transmit_message.tx_efid = 0xaabb;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_EXTENDED;
    transmit_message.tx_dlen = 8;
    transmit_message.tx_data[0] = 0xA0;
    transmit_message.tx_data[1] = 0xA1;
    transmit_message.tx_data[2] = 0xA2;
    transmit_message.tx_data[3] = 0xA3;
    transmit_message.tx_data[4] = 0xA4;
    transmit_message.tx_data[5] = 0xA5;
    transmit_message.tx_data[6] = 0xA6;
    transmit_message.tx_data[7] = 0xA7;

    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);

	// 开始发送数据
	transmit_mailbox = can_message_transmit(CANX, &transmit_message);
	/* waiting for transmit completed */
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CANX, transmit_mailbox)) && (0 != timeout))
	{
		timeout--;
	}

	// 等待中断接受完毕
	// if(SET == receive_flag)
	// {
	// 	receive_flag = RESET;
	// 	printf("\r\n can receive data:");
	// 	for(i = 0; i < receive_message.rx_dlen; i++)
	// 	{
	// 		printf(" %02x", receive_message.rx_data[i]);
	// 	}
	// }
}



