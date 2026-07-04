import socket
import sys

# CONFIGURAZIONE
HOST = '127.0.0.1'
PORT = 1234

def monitor_serial():
    # Creiamo un socket TCP/IP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Permette di riutilizzare la porta immediatamente dopo la chiusura
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"[*] Server in ascolto su {HOST}:{PORT}...")
        print("[*] Avvia QEMU nel secondo terminale per iniziare...")
        
        # Il server si blocca qui in attesa che QEMU si connetta
        client_socket, client_address = server_socket.accept()
        print(f"[+] QEMU connesso da: {client_address}")
        
        # Creiamo un file-like object dal socket per poter usare readline() comodamente
        buffer = client_socket.makefile('r', encoding='utf-8', errors='ignore')
        
        while True:
            # Legge una riga inviata da QEMU (fino al carattere \n)
            linea = buffer.readline()
            
            if not linea:
                print("[*] Connessione chiusa da QEMU.")
                break
                
            testo_decodificato = linea.strip()
            if testo_decodificato:
                print(f"python reply: {testo_decodificato}")
                
    except KeyboardInterrupt:
        print("\n[*] Monitoraggio interrotto dall'utente.")
    except Exception as e:
        print(f"\n[!] Errore: {e}")
    finally:
        server_socket.close()
        print("[*] Server spento.")

if __name__ == "__main__":
    monitor_serial()