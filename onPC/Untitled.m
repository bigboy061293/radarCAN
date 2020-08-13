port        = 2992;
 payloadData = cell(100,1);
while 1
   % get the message/payload only assuming a max size of 200 bytes
   [msg,~] = judp('RECEIVE',port,32);
   % save the payload to the array
   payloadData{k} = msg;
   % convert the message to ASCII and print it out
   fprintf('%s\n',char(msg)');
 end