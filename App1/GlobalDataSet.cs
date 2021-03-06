﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;

namespace CanTest
{
    class GlobalDataSet
    {
        private int mAX_WAIT_TIME = 800;
        private int TIME_SLOW_DOWN_CODE = 1;
        private bool spi_not_initialized = true;
        private GpioPin mCP2515_PIN_CS_SENDER, mCP2515_PIN_INTE_SENDER;
        private GpioPin mCP2515_PIN_CS_RECEIVER, mCP2515_PIN_INTE_RECEIVER;
        private GpioPin rEQUEST_DATA_EXECUTOR_0, rEQUEST_DATA_EXECUTOR_1, sTART_PIN_OUT, rEQUEST_DATA_HANDSHAKE_EXECUTOR_0, rEQUEST_DATA_HANDSHAKE_EXECUTOR_1, rSAVE_ACT_ENC_POS;
        private MCP2515 mcp2515;
        private SpiDevice spiDevice;
        private Logic_Mcp2515_Sender logic_Mcp2515_Sender;
        private Logic_Mcp2515_Receiver logic_Mcp2515_Receiver;
        private bool debugMode;
        private string HOST_IP = "192.168.0.22";
        private string HOST_PORT_RECEIVE = "4002";
        private string HOST_PORT_SEND = "4001";
        private byte[] currentMotorAngle = new byte[2];
        private byte[] currentMotorSollValue = new byte[2];
        private bool saveActEncPos = false;


        private ActionStates robotPending = ActionStates.init;

        // CONTENT
        // 0. byte: Number of the task 
        // 1. byte: ID of the motor to move
        // 2-3. byte: Velocity to move the motor to the specified position
        // 4-5. byte: Endposition for the specific motor 
        // 6. byte: Direction of the motor
        // 7. byte: 
        private byte[] soll_control_values = new byte[8];

        // CONTENT
        // 0. byte: Number of the task 
        // 1. byte: ID of the motor to move
        // 2-3. byte: Velocity to move the motor to the specified position
        // 4-5. byte: Endposition for the specific motor 
        // 6. byte: Direction of the motor
        // 7. byte: 
        private byte[] ist_control_values = new byte[8];

        public enum Soll_ControlDataAssignment
        {
            soll_task_no,
            soll_motor_id,
            soll_motor_vel_1,
            soll_motor_vel_2,
            soll_motor_angle_1,
            soll_motor_angle_2,
            soll_motor_dir
        };

        public enum Ist_ControlDataAssignment
        {
            ist_task_no,
            ist_motor_id,
            ist_motor_vel_1,
            ist_motor_vel_2,
            ist_motor_angle_1,
            ist_motor_angle_2,
            ist_encoder_dir
        };

        public enum ActionStates
        {
            init,
            complete,
            pending
        };

        public GlobalDataSet()
        {
            debugMode = false;
            mcp2515 = new MCP2515();
            logic_Mcp2515_Sender = new Logic_Mcp2515_Sender(this);
            logic_Mcp2515_Receiver = new Logic_Mcp2515_Receiver(this);
        }

        public int MAX_WAIT_TIME
        {
            get
            {
                return mAX_WAIT_TIME;
            }

            set
            {
                mAX_WAIT_TIME = value;
            }
        }

        public async void init_mcp2515_task()
        {
            await Task.Run(() => init_mcp2515());
        }

        private void init_mcp2515()
        {
            logic_Mcp2515_Receiver.init_mcp2515_receiver();
        }

        public bool Spi_not_initialized
        {
            get
            {
                return spi_not_initialized;
            }

            set
            {
                spi_not_initialized = value;
            }
        }

        public GpioPin do_mcp2515_cs_sen
        {
            get
            {
                return mCP2515_PIN_CS_SENDER;
            }

            set
            {
                mCP2515_PIN_CS_SENDER = value;
            }
        }

        public GpioPin di_mcp2515_int_sen
        {
            get
            {
                return mCP2515_PIN_INTE_SENDER;
            }

            set
            {
                mCP2515_PIN_INTE_SENDER = value;
            }
        }

        public GpioPin do_mcp2515_cs_rec
        {
            get
            {
                return mCP2515_PIN_CS_RECEIVER;
            }

            set
            {
                mCP2515_PIN_CS_RECEIVER = value;
            }
        }

        public GpioPin di_mcp2515_int_rec
        {
            get
            {
                return mCP2515_PIN_INTE_RECEIVER;
            }

            set
            {
                mCP2515_PIN_INTE_RECEIVER = value;
            }
        }

        public SpiDevice SPIDEVICE
        {
            get
            {
                return spiDevice;
            }

            set
            {
                spiDevice = value;
            }
        }

        public Logic_Mcp2515_Receiver LOGIC_MCP2515_RECEIVER
        {
            get
            {
                return logic_Mcp2515_Receiver;
            }

            set
            {
                logic_Mcp2515_Receiver = value;
            }
        }

        public Logic_Mcp2515_Sender LOGIC_MCP2515_SENDER
        {
            get
            {
                return logic_Mcp2515_Sender;
            }

            set
            {
                logic_Mcp2515_Sender = value;
            }
        }

        public GpioPin do_startAcquisition
        {
            get
            {
                return REQUEST_DATA_EXECUTOR_0;
            }

            set
            {
                REQUEST_DATA_EXECUTOR_0 = value;
            }
        }

        public GpioPin REQUEST_DATA_EXECUTOR_1
        {
            get
            {
                return rEQUEST_DATA_EXECUTOR_1;
            }

            set
            {
                rEQUEST_DATA_EXECUTOR_1 = value;
            }
        }

        public GpioPin START_PIN_OUT
        {
            get
            {
                return sTART_PIN_OUT;
            }

            set
            {
                sTART_PIN_OUT = value;
            }
        }

        public GpioPin REQUEST_DATA_HANDSHAKE_EXECUTOR_0
        {
            get
            {
                return rEQUEST_DATA_HANDSHAKE_EXECUTOR_0;
            }

            set
            {
                rEQUEST_DATA_HANDSHAKE_EXECUTOR_0 = value;
            }
        }

        public GpioPin REQUEST_DATA_HANDSHAKE_EXECUTOR_1
        {
            get
            {
                return rEQUEST_DATA_HANDSHAKE_EXECUTOR_1;
            }

            set
            {
                rEQUEST_DATA_HANDSHAKE_EXECUTOR_1 = value;
            }
        }



        internal MCP2515 Mcp2515
        {
            get
            {
                return mcp2515;
            }

            set
            {
                mcp2515 = value;
            }
        }

        public bool DebugMode
        {
            get
            {
                return debugMode;
            }

            set
            {
                debugMode = value;
            }
        }

        public void writeSimpleCommandSpi(byte command, GpioPin cs_pin)
        {
            cs_pin.Write(GpioPinValue.Low);
            spiDevice.Write(new byte[] { command });
            cs_pin.Write(GpioPinValue.High);
            //Task.Delay(-1).Wait(TIME_SLOW_DOWN_CODE);
        }

        public byte[] readSimpleCommandSpi(byte registerAddress, GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[1];

            cs_pin.Write(GpioPinValue.Low);
            spiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_READ, registerAddress });
            spiDevice.Read(returnMessage);
            cs_pin.Write(GpioPinValue.High);
            //Task.Delay(-1).Wait(TIME_SLOW_DOWN_CODE);

            return returnMessage;
        }

        public byte[] readSimpleCommandSpi_v2(byte registerAddress, GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[7];
            byte[] returnMessageTemp = new byte[1];

            cs_pin.Write(GpioPinValue.Low);
            spiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_READ, registerAddress });
            for (int i = 0; i < 7; i++)
            {
                spiDevice.Read(returnMessageTemp);
                returnMessage[i] = returnMessageTemp[0];
            }
            cs_pin.Write(GpioPinValue.High);

            return returnMessage;
        }

        public byte[] readSimpleCommandSpi_v3(byte bufferId, GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[mcp2515.MessageSizeToMcp];
            byte[] returnMessageTemp = new byte[1];

            cs_pin.Write(GpioPinValue.Low);
            spiDevice.Write(new byte[] { bufferId });
            for (int i = 0; i < mcp2515.MessageSizeToMcp; i++)
            {
                spiDevice.Read(returnMessageTemp);
                returnMessage[i] = returnMessageTemp[0];
                //Debug.WriteLine("returnMessage[" + i + "] " + returnMessage[i]);
            }
            cs_pin.Write(GpioPinValue.High);

            return returnMessage;
        }

        public byte mcp2515_execute_read_command(byte registerToRead, GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[1];
            byte[] sendMessage = new byte[1];

            // Enable device
            cs_pin.Write(GpioPinValue.Low);

            // Write spi instruction read  
            sendMessage[0] = mcp2515.SPI_INSTRUCTION_READ;
            spiDevice.Write(sendMessage);

            // Write the address of the register to read
            sendMessage[0] = registerToRead;
            spiDevice.Write(sendMessage);
            spiDevice.Read(returnMessage);

            // Disable device
            cs_pin.Write(GpioPinValue.High);
            //Task.Delay(-1).Wait(TIME_SLOW_DOWN_CODE);

            return returnMessage[0];
        }

        public void mcp2515_execute_write_command(byte[] spiMessage, GpioPin cs_pin)
        {
            // Enable device
            cs_pin.Write(GpioPinValue.Low);

            // Write spi instruction write  
            spiDevice.Write(new byte[] { mcp2515.SPI_INSTRUCTION_WRITE });
            spiDevice.Write(spiMessage);
            cs_pin.Write(GpioPinValue.High);
            //Task.Delay(-1).Wait(TIME_SLOW_DOWN_CODE);
        }

        public byte executeReadStateCommand(GpioPin cs_pin)
        {
            byte[] returnMessage = new byte[1];
            byte[] sendMessage = new byte[1];

            // Enable device
            cs_pin.Write(GpioPinValue.Low);

            // Write spi instruction read  
            sendMessage[0] = mcp2515.SPI_INSTRUCTION_READ_STATUS;
            spiDevice.Write(sendMessage);
            spiDevice.Read(returnMessage);

            // Disable device
            cs_pin.Write(GpioPinValue.High);
            //Task.Delay(-1).Wait(TIME_SLOW_DOWN_CODE);

            return returnMessage[0];
        }

        private bool mcpExecutionIsActive = false;

        private bool stopAllOperations = false;

        public string[] sendBuffer = {"", "", "", "", "", "", "", "", "", "",
                                        "", "", "", "", "", "", "", "", "", "",
                                        "", "", "", "", "", "", "", "", "", "",
                                        "", "", "", "", ""};
        public bool[] bufferState = {false, false, false, false, false, false, false, false, false, false,
                                        false, false, false, false, false, false, false, false, false, false,
                                        false, false, false, false, false, false, false, false, false, false,
                                        false, false, false, false, false};

        public bool clientIsConnected
        {
            get
            {
                return mcpExecutionIsActive;
            }

            set
            {
                mcpExecutionIsActive = value;
            }
        }

        public bool StopAllOperations
        {
            get
            {
                return stopAllOperations;
            }

            set
            {
                stopAllOperations = value;
            }
        }

        public string HostIp
        {
            get
            {
                return HOST_IP;
            }

            set
            {
                HOST_IP = value;
            }
        }

        public string HostPortSend
        {
            get
            {
                return HOST_PORT_SEND;
            }

            set
            {
                HOST_PORT_SEND = value;
            }
        }

        public string HostPortReceive
        {
            get
            {
                return HOST_PORT_RECEIVE;
            }

            set
            {
                HOST_PORT_RECEIVE = value;
            }
        }

        public byte[] CurrentMotorAngle
        {
            get
            {
                return currentMotorAngle;
            }

            set
            {
                currentMotorAngle = value;
            }
        }

        public byte[] CurrentMotorControlData
        {
            get
            {
                return currentMotorSollValue;
            }

            set
            {
                currentMotorSollValue = value;
            }
        }


        public byte[] Incoming_DataPackage
        {
            get
            {
                return soll_control_values;
            }

            set
            {
                soll_control_values = value;
            }
        }

        public byte[] Outgoing_DataPackage
        {
            get
            {
                return ist_control_values;
            }

            set
            {
                ist_control_values = value;
            }
        }

        public string[] getSendBuffer()
        {
            return sendBuffer;
        }

        public void setSendBuffer(string[] sendBuffer)
        {
            this.sendBuffer = sendBuffer;
        }

        public bool[] getBufferState()
        {
            return bufferState;
        }

        public void setBufferState(bool[] bufferState)
        {
            this.bufferState = bufferState;
        }

        public ActionStates PendingAction
        {
            get
            {
                return robotPending;
            }

            set
            {
                robotPending = value;
            }
        }

        public GpioPin REQUEST_DATA_EXECUTOR_0
        {
            get
            {
                return rEQUEST_DATA_EXECUTOR_0;
            }

            set
            {
                rEQUEST_DATA_EXECUTOR_0 = value;
            }
        }

        public GpioPin do_saveActEncPos
        {
            get
            {
                return rSAVE_ACT_ENC_POS;
            }

            set
            {
                rSAVE_ACT_ENC_POS = value;
            }
        }

        public bool SaveActEncPos
        {
            get
            {
                return saveActEncPos;
            }

            set
            {
                saveActEncPos = value;
            }
        }
    }
}
