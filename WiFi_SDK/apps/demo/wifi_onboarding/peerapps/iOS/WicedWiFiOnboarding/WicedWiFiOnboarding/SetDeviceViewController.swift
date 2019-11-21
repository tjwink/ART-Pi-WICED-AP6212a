/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

import UIKit

protocol SetDeviceViewControllerDelegate {
    func onDeviceAPConnected()
}

class SetDeviceViewController: UIViewController {
    
    var delegate: SetDeviceViewControllerDelegate?
    var homeAPnotset = 0;
    var homeAPset = 1;
    var deviceAPset = 2;
    var state = 0;

    @IBOutlet weak var homeAP: UILabel!
  
    func applicationDidBecomeActive(notification: NSNotification) {
        print("applicationDidBecomeActive")
        if state == homeAPnotset {
            checkforWifi()
        }
        
    }
    
    override func viewDidLoad() {

        super.viewDidLoad()
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(applicationDidBecomeActive),
            name: NSNotification.Name.UIApplicationDidBecomeActive,
            object: nil)
        
        let homeapssid = DataManager.sharedInstance().homeAP
        homeAP.text = "\(homeapssid)"
        
        
     
    }
    
    override func viewDidAppear(_ animated: Bool) {
        print("viewDidAppear")

    }
    
  
    
    override func viewWillAppear(_ animated: Bool) {
        print("viewWillAppear")
        super.viewWillAppear(animated)
        
    }

    @IBAction func onDoneButtonClick(_ sender: Any) {
        
        if state == homeAPnotset {
            checkforWifi()
        }
        
        if state == homeAPset {
            let ssidName1 = SSIDNetwork()
            if ssidName1.getSSID() != "WICED DEVICE AP" {
                DataManager.sharedInstance().deviceAP = ssidName1.getSSID()
            }
            notifyCallingViewController();
        }
        
    }

    func notifyCallingViewController() {
        print("onDeviceAPConnected")
        self.delegate?.onDeviceAPConnected()
    }
    
    func checkforWifi() {
        
        let ssidName = SSIDNetwork()
        
        if let temp = ssidName.getSSID() {
            print("connected to network \(temp)")
            DataManager.sharedInstance().homeAP = temp;
            state = homeAPset;
            homeAP.text = "\(temp)"
            
        } else {
            
            print("NOT CONNECTED TO NETWORK.KINDLY CONNECT!")
            showPopUp()
        }
    }
    
    
    func showPopUp() {
        let alertController = UIAlertController (title: "Not connected to Wi-Fi", message: "Go to Settings and connect to Home AP", preferredStyle: .alert)
        
        let settingsAction = UIAlertAction(title: "Settings", style: .default) { (_) -> Void in
            
            if UIApplication.shared.canOpenURL(URL(string:"App-Prefs:root=WIFI")!) {
                
                UIApplication.shared.open(URL(string:"App-Prefs:root=WIFI")!, completionHandler: { (success) in
                    print("Settings opened: \(success)") // Prints true
                    
                    
                })
            }
        }
        alertController.addAction(settingsAction)
        let cancelAction = UIAlertAction(title: "Cancel", style: .default, handler: nil)
        alertController.addAction(cancelAction)
        
        present(alertController, animated: true, completion: nil)
    }
    



}
