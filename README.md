# TinyWireSetup
Working I2c bitbang software with TinyWire slave node
I2C library setup for ATmega and ATTiny
--------------------
Copyright (c) 2022 ChrisHul @ github.com

Working library setup using "TinyWire-master" library composed 	by "lucullus" together with Bit Bang I2C library
     developed by Larry Bank.

Full credit is given to above mentioned sources. Both 
     libraries have undergone thorough overhaul through 
     testing and adaption to own needs.

Testing conditions:
     Master: Arduino Nano running BitBangI2c library at 
             bitrate 100000,
     Slave: ATtiny84 with TinyWire library for slave
            operation running @8Mhz internal clock
     Sending: ~5 bytes
     Receiving: ~5 bytes

Observations:
     ATMega and ATTiny have two different types of serial 
     interfaces, the latter being a "budget" type.
     ATTiny's USI only has two phases of serial shifting,
     whereas ATMega has a multiple phase interface.
     Therefore ATTiny can never be fully I2c compliant.
     The major issue with the "budget" solution is the ACK
     feedback in the transfer operation, which has to be
     dealt with by the software. The I2c protocol does not
     allow SCL (clock) stretching between data transfer and
     acknowledge and the processor is required to setup
     the acknowledge in a very limited period of time.
     If the processor is held-up by other processes (read
     other interrupts) it will fail to respond to 
     requirements.

Solution:
     Adapted bitbang operation at master which will allow
     SCL stretching at any time. This will allow the slave
     to stretch the clock the time needed to prepare data.
     Enhanced TinyWire software allowing full SCL stretching

Results:
     Acceptable bitrate of 100000 possible. Failure rate
     mainly when sending from master to slave of 0.1%.
     Failure consists of byte being shifted one step to 
     the left (data<<1). This happens randomly in the
     data stream and does not affect the rest of the bytes
     in the stream. Trial and error have shown extending 
     delay between data and ACK in master send alleviates
     the problem. No other clue to the reason behind it, also
     not why it only happens on master send and not receive.

Recommendations:
     This software has been released in view of the problems
     people have encountered using the ATTiny USI. With an
     additional CRC and handshake layer it should provide
     a reliable tranfer platform for multiple processor
     setups involving the ATTiny.

See: Source files for modification details.




// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
