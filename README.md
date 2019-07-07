# M17_ANL  
M17 analyser. May be used for receiving and sending packets over the air (half-duplex).  
  
# Usage  
Communication is realised via UART, 230400 baud, 8 bits of data, no parity, 1 stop bit, no handshake. End of line is \r\n.  
Commands:  
- AT+FREQ?  - shows current frequency (in Hz), default = 439575000  
- AT+PWR?   - shows current power setting, default = 0  

- AT+FREQ=x - sets current frequency (in Hz), range 420..460MHz (inclusively). CAUTION: transmitting signals beyond the amateur band (430..440MHz) may be prohibited by local law.  
- AT+PWR=x  - sets current power setting, range 0 to 127, inclusively. Please refer to the Si4463-C2 datasheet for more information on this.  

- AT+FRAME=x  - sends a 97-byte frame. Replace x with a 97-byte long string ended with \r\n. The device also sends this string upon receiving a correct frame. It also contains 97 bytes of data after the '=' sign. EOL is \r\n as mentioned above.
