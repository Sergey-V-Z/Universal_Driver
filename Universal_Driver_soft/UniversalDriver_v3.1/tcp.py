from socket import *
import sys

# Адрес центральной платы
host = '192.168.0.4'
port = 81
addr = (host, port)

tcp_socket = socket(AF_INET, SOCK_STREAM)
tcp_socket.connect(addr)

#data = input('write to server: ')
#if not data:
#    tcp_socket.close()
#    sys.exit(1)

# encode - перекодирует введенные данные в байты, decode - обратно
#data = str.encode(data)

# Флаги для формирования сообщения
comandFlag = "C"
addresFlag = "A"
dataFlag = "D"
endFlag = "x"

# комманды
cmd1 = "11"
cmd2 = "12"
cmd3 = "13"

# адрес переменных
#Addr1 = "4"
#Addr2 = "3"
#Addr3 = "2"

# данные для записи
wdata1 = "1"
wdata2 = "500"
wdata3 = "0"

# data = int(data)
# собираем сообщение
data = comandFlag
data += cmd1
data += addresFlag
data += dataFlag
data += wdata1
data += endFlag

data += comandFlag
data += cmd2
data += addresFlag
data += dataFlag
data += wdata2
data += endFlag

data += comandFlag
data += cmd3
data += addresFlag
data += dataFlag
data += wdata3
data += endFlag

tcp_socket.send(data.encode())
#data = bytes.decode(data)
data = tcp_socket.recv(1024)
print(data)

tcp_socket.close()
