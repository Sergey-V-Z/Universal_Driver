from socket import *
import sys

# Адрес центральной платы
host = '192.168.0.11'
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
cmd1 = "17"
cmd2 = "1"
cmd3 = "2"

# адрес переменных
Addr1 = "4"
Addr2 = "3"
Addr3 = "2"

# данные для записи
wdata1 = "44"
wdata2 = "55"
wdata3 = "1024"

# data = int(data)
# собираем сообщение
data = comandFlag
data += cmd1
data += addresFlag
data += Addr1
data += dataFlag
data += wdata1
data += endFlag

data += comandFlag
data += cmd2
data += addresFlag
data += Addr2
data += dataFlag
data += wdata2
data += endFlag

data += comandFlag
data += cmd3
data += addresFlag
data += Addr3
data += dataFlag
data += wdata3
data += endFlag

tcp_socket.send(data.encode())
#data = bytes.decode(data)
data = tcp_socket.recv(1024)
print(data)

tcp_socket.close()
