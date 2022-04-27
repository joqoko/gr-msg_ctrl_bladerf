# Message passing controled BladeRF source block
This repo contains a modified bladerf source block that can be configured online with its message passing port. There are also a few additional blocks for debugging and demostration.

## input data type format

Currently the msg_bladerf_src block supports 6 types of online reconfigurations. The PMT element contains two fields, namely the key word and the value. The key word is a string that contains the targeted parameter to be configured. For some simple reconfiurations like center frequency and bandwidth, the value is an number, but for channel related reconfigurations it is a pair. The car part is the value and the cdr part indicates the channel.

1. Center frequency  
   key word: [freq]  
   Format: {cmd:"freq", value:2411500000}

2. Bandwidth  
   key word: [bw] 
   Format: {cmd:"bw", value:10000000}

3. Gain  
   key word: [gain]  
   Format: {cmd:"gain", value:(40, 0)}

4. Gain mode  
   key word: [gainmode]  
   Format: {cmd:"gainmode", value:(1, 0)}

5. External reference  
   key word: [external_ref]  
   Format: {cmd:"external_ref", value:0}

7. Biastee  
   key word: [biastee] 
   Format: {cmd:"biastee", value:0}


