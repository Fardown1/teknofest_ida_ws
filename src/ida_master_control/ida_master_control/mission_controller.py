
#/* ******************************************************* 
# * Filename		:	mission_controller.py
# * Author		    :	Berkin		
# * Date			:	25.11.2025  // 01.09 latest..
# * Description	    :	Brain 101 // idle halde başlar
#/ * ******************************************************/


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
import time


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # Degiskenleri burda tanimliyorum
        self.durum = 0      # 0: Bekleme, 1: Hazirlik, 2: Parkur1...
        self.baslangic = 0
        self.sure_siniri = 1200
        
        
        # Publisherlar
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/ida/mission_status', 10)


        # Subscriberlar
        self.create_subscription(Bool, '/ida/start_command', self.baslat_callback, 10)
        self.create_subscription(Bool, '/ida/emergency_stop', self.acil_durum_callback, 10)
        self.create_subscription(Float32, '/ida/battery_level', self.pil_callback, 10)


        # Loop
        self.timer = self.create_timer(0.1, self.dongu)
        
        print("\n\n Gorev Kontrolcusu Basladi \n")


    def dongu(self):
        
        # Sure kontrolu
        if self.durum != 0 and self.durum != 5 and self.durum != 99:
            gecen = time.time() - self.baslangic
            
            if gecen > self.sure_siniri:
                self.durum = 5
                print("\n Sure Doldu \n")


        # Durum Makinasi (State Machine)
        
        if self.durum == 0:
            # Bekleme modu
            pass
            
            
        elif self.durum == 1:
            # Asama 1: Hazirlik
            self.get_logger().info('Hazirlik yapiliyor...')
            
            time.sleep(2)
            
            self.durum = 2
            print("\n Gecis yapildi -> Parkur 1 \n")
            
            
        elif self.durum == 2:
            # Asama 2: Waypoint Navigasyonu
            # Buraya navigasyon kodlari gelecek
            pass
            
            
        elif self.durum == 3:
            # Asama 3: Engelden Kacma
            pass
            
            
        elif self.durum == 4:
            # Asama 4: Kamikaze
            pass
            
            
        elif self.durum == 5:
            # Gorev Bitti
            self.durdur()
            
            
        elif self.durum == 99:
            # ACIL DURUM
            self.get_logger().error('ACIL DURUM!')
            self.durdur()


        # Durumu yayinla
        msg = String()
        msg.data = str(self.durum)
        self.status_pub.publish(msg)


    def baslat_callback(self, msg):
        
        if msg.data == True:
            if self.durum == 0:
                
                print("\n Baslat komutu geldi \n")
                self.baslangic = time.time()
                self.durum = 1


    def acil_durum_callback(self, msg):
        
        if msg.data == True:
            self.durum = 99


    def pil_callback(self, msg):
        
        pil = msg.data
        if pil < 11.0:
            print("\n Pil kritik seviyede \n")


    def durdur(self):
        
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        
        self.cmd_vel_pub.publish(stop)


def main(args=None):
    
    rclpy.init(args=args)
    
    node = MissionController()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()