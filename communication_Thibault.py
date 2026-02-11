import threading
import socket
import time

class Communication:
    def __init__(self,port=29200,mission_start_time=0,mission_duration=100):
        self.local_server_ip = "0.0.0.0"
        self.our_ddboat_ip = "172.20.25.209"  # ligne ou vous mettez votre adresse ip
        self.distant_server_ip = "172.20.25.207"  # server hostname or IP address
        self.log_server_ip = "172.20.254.254"  # server hostname or IP address
        self.port = port  # server port number has to be > 1024
        self.log_server_port = 6969
        self.end_of_mission_message="fin_mission"
        self.mission_start_time = mission_start_time
        self.mission_duration = mission_duration
        self.broadcast_period = 1 # 1Hz
        self.time_out = self.broadcast_period + 10
        self.t0 = time.time()
        #self.tf = self.t0+mission_duration

    def parse_send_data(self,x,y,phi):
        """
        Formatage des donnees au format commun pour l'envoie depuis le serveur
        """
        return str(x)+"|"+str(y)+"|"+str(phi)

    def parse_log_data(self,id,lon,lat,j):
        """
        Formatage des donnees au format commun pour l'envoie depuis le serveur
        """
        return str(id)+"|"+str(lon)+"|"+str(lat)+"|"+str(j)

    def parse_received_data(self,data):
        """
        Fonction de decodage des donnees au format stadard, recues par un des clients
        """
        try:
            decoded_data = data.split("|")
            decoded_data[0] = float(decoded_data[0])
            decoded_data[1] = float(decoded_data[1])
            decoded_data[2] = float(decoded_data[2])
        except:
            print("Error in decoding data : ", data)
            decoded_data = [0,0,0]
        return decoded_data

    def handle_client(self, client_socket, addr):
        try:
            while True:
                # receive and print client messages
                ddboat_data = self.parse_send_data(0,0,0) #################### Ici doit etre lie a notre ddboat pour connaitre ses donnes
                client_socket.send(ddboat_data.encode("utf-8")[:1024])

                # si la mission se fini, envoie du message de fin de la connection
                if time.time()-self.t0>=self.mission_duration:
                    print("Sending end of communication with",addr)
                    client_socket.send(self.end_of_mission_message.encode("utf-8"))
                    break
                time.sleep(self.broadcast_period)
        except Exception as e:
            print("Error when hanlding client:",e)

        client_socket.close()
        print("Connection to client (",addr[0],":",addr[1],") closed")

    def run_server(self):
        """
        Ouverture du serveur du DDBoat, dont le but est de broadcast ses donnees aux autres
        """
        # create a socket object
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # bind the socket to the host and self.port
            server.bind((self.local_server_ip, self.port))
            # listen for incoming connections
            server.listen(20)
            print("Listening on",self.local_server_ip,":",self.port)
            server.settimeout(self.time_out)
            while time.time() - self.t0 < self.mission_duration:
                try:
                    client_socket, addr = server.accept()
                    print("Accepted connection from", addr)

                    thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, addr),
                        daemon=True
                    )
                    thread.start()

                except socket.timeout:
                    continue  # check mission time again

        except Exception as e:
            print("Error:", e)
        print("Closing server...[5s remaining]")
        time.sleep(5)
        print("...Server closed")

        server.close()

    def run_client(self,ddboat_number="01"):
        try:
            # create a socket object
            peer_ip = self.distant_server_ip+ddboat_number
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Connection demand sent to DDBoat",ddboat_number, "at ip",peer_ip)
            # establish connection with server
            for attempt in range(10):
                try:
                    client.connect((peer_ip, self.port))
                    print("Connected to DDBoat",ddboat_number)
                    break
                except ConnectionRefusedError:
                    print("Server not ready, retrying...")
                    time.sleep(1)
            else:
                print("Failed to connect after retries.")
                return

            client.settimeout(self.time_out)
            while time.time() - self.t0 < self.mission_duration:
                # receive message from the server
                try:
                    response = client.recv(1024)
                    response = response.decode("utf-8")
                    if response==self.end_of_mission_message:# if server closes, close connection
                        break
                    else:
                        decoded_data = self.parse_received_data(response)
                        print("DDBoat",ddboat_number,"at coords",decoded_data[0],",",decoded_data[1],"with phase",decoded_data[2])
                except socket.timeout:
                    print("No response, stopping client.")
                    break

            # close client socket (connection to the server)
            client.close()
            print("Connection to DDBoat",ddboat_number,"closed")
        except Exception as e:
            print("DDBoat",ddboat_number,"is unreachable. Error :",e)

    def run_log_client(self):
        try:
            # Open UDP client socket
            UDPClientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            while time.time() - self.t0 < self.mission_duration:
                # sending log data to log server
                log_data = self.parse_log_data(0,0,0,0) # ligne a connecter au reste de votre programme pour envoyer vos donnees
                log_data = str.encode(log_data)
                UDPClientSocket.sendto(log_data,(self.log_server_ip, self.log_server_port))
                time.sleep(self.broadcast_period)
            # close client socket (connection to the server)
            UDPClientSocket.close()
            print("LOG SERVER CLIENT closed")
        except:
            print("LOG SERVER is unreachable.")

    def run_communication(self,number_of_peers=1):
        thread_server = threading.Thread(target=self.run_server)
        thread_server.start()
        thread_log_client = threading.Thread(target=self.run_log_client)
        thread_log_client.start()
        time.sleep(2)
        for i in range(1,number_of_peers):
            if len(str(i))==2:
                ddboat_number=str(i)
            else:
                ddboat_number="0"+str(i)
            if not (self.distant_server_ip+ddboat_number==self.our_ddboat_ip): # on ne veut pas se connecter a nous-meme
                thread_client = threading.Thread(target=self.run_client,args=(ddboat_number,),daemon=True)
                thread_client.start()

        thread_server.join()
        print("Mission finished.")


if __name__=="__main__":
    communicant = Communication(port=29200,mission_start_time=0,mission_duration=500)
    communicant.run_communication(number_of_peers=15)

