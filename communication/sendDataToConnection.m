function sendDataToConnection(client, data)
write(client, data(:), "uint8");
end
