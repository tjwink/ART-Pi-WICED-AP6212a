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
import CoreBluetooth
import Alamofire

protocol WirelessSetupViewControllerDelegate {
    func setUpComplete(_data: Bool)
}

class WirelessSetupViewController: UIViewController, UITextFieldDelegate{
    
    
    @IBOutlet weak var passwordText: UITextField!
    @IBOutlet weak var homeAp: UILabel!
    @IBOutlet weak var deviceAP: UILabel!

    @IBOutlet weak var Switch: UISwitch!


    var securityMode : Int?
    var ssidName : String?
    var passphraseToken : String?
    var delegate: WirelessSetupViewControllerDelegate?
    var timer = Timer()
    var serverTrustPolicy : ServerTrustPolicy?
    var serverTrustPolicies = [String : ServerTrustPolicy]()
    var afManager : SessionManager?
    var items: [String] = ["WPA2-TKIP", "WPA2-AES", "WPA2-MIXED"]
    var enforceSsl : Bool = true
    
    override func viewDidLoad() {
        
        super.viewDidLoad()
        
        deviceAP.text = "\u{2022} \(DataManager.sharedInstance().deviceAP)";
        homeAp.text = "\u{2022} \(DataManager.sharedInstance().homeAP)";
        
        let deviceAptxt = "WICED DEVICE AP"
      
        if deviceAptxt != "WICED DEVICE AP" {
            deviceAP.text = "\u{2022} \(deviceAptxt)";
            //deviceName.text = devName
        }

        let ssidName1 = SSIDNetwork()
        ssidName = "HOME AP"
        Switch.isOn = false;
        Switch.tintColor = UIColor.white
        Switch.backgroundColor = UIColor.white
        Switch.layer.cornerRadius = 16.0
        let temp = ssidName1.getSSID()!
        if temp != nil {

            deviceAP.text = "\u{2022} \(temp)";
            ssidName = temp
            
        }
        let tap: UITapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(WirelessSetupViewController.dissmissKeyboard));
        view.addGestureRecognizer(tap)
        passwordText.layer.sublayerTransform = CATransform3DMakeTranslation(10, 0, 0)

    }

    @IBAction func Connect(_ sender: AnyObject) {
        print("Connect :")
        print(passwordText.text!)
        let ssidName1 = SSIDNetwork()
        ssidName1.getSSID()
        DataManager.sharedInstance().homeAPpwd = passwordText.text!
        
        configureAlamoFireSSLPinning()
        bypassURLAuthentication()
        makePostRequest()

    }
 
    @IBAction func showPasswordChange(_ sender: AnyObject) {
        if Switch.isOn {
            passwordText.resignFirstResponder()
            passwordText.isSecureTextEntry = false
        } else {
            Switch.tintColor = UIColor.white
            Switch.backgroundColor = UIColor.white
            Switch.layer.cornerRadius = 16.0
            passwordText.isSecureTextEntry = true
        }
    }

    func dissmissKeyboard() {
        view.endEditing(true)
    }

    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        // Hide the keyboard.
        textField.resignFirstResponder()
        return true
    }
    
    @IBAction func passwordFiledTouchOutside(_ sender: AnyObject) {
        passwordText.endEditing(true)
        passwordText.resignFirstResponder()
    }
    @IBAction func PasswordEditingFinished(_ sender: AnyObject) {
        passwordText.resignFirstResponder()
    }
    
    
    func notifyCallingViewController(val:Bool) {
        print("notifyCallingViewController setUpComplete")
        self.delegate?.setUpComplete(_data: val)
    }
    
    func change_ui(val:Int) {
        print("UI got val:\(val)")
        if (val == 0)
        {
            notifyCallingViewController(val : false)
        }
        else
        {
            notifyCallingViewController(val : true)
        
        }
    }
    
    func makeConfirmRequest() {
        print( "Triggering Confirm-Onboarding PUT request")

        afManager?.request("http://www.wiced.com/confirm-onboarding", method: .put)
            .responseData { response in
                
                if (response.result.isSuccess)
                {
                    print("confirm onboarding success", response.response?.statusCode)
                } else {
                    print("confirm onboarding error", response.response?.statusCode)
                }
        }
    }

    
    public func URLsession(session: URLSession, didReceiveChallenge challenge: URLAuthenticationChallenge,
                           completionHandler: (URLSession.AuthChallengeDisposition, URLCredential?) -> Void) {
        // Verify that challenge did come from our API server.
        if protectionSpaceVerified(protectionSpace: challenge.protectionSpace){
            let credentials = URLCredential(trust: challenge.protectionSpace.serverTrust!)
            completionHandler(URLSession.AuthChallengeDisposition.useCredential, credentials)
        }
        else{
            completionHandler(URLSession.AuthChallengeDisposition.cancelAuthenticationChallenge, nil)
        }
    }
    
    func protectionSpaceVerified(protectionSpace:URLProtectionSpace)->Bool{
        var verified = true
//        verified = verified && (protectionSpace.host.compare("www.wiced.com",
//                                                             options: [.CaseInsensitiveSearch], range: nil, locale: nil) == .OrderedSame)
        if (enforceSsl){
            verified = verified && (protectionSpace.port == 443)
        }
        return verified
    }
    
    
    func makeGetRequest() {
     
        var result = 0;
        let req2 = self.afManager?.request("https://www.wiced.com/config-status", method: .get)
            .responseData {
                response in
                print(response.description)
                print(response.debugDescription)
                print(response.request)  // original URL request
                print(response.response) // URL response
                print(response.data)     // server data
                print(response.result)   // result of response serialization
                print ("response value as config\(response.value)")
                print("response result value as config\(response.result.value)")
                let array_int = UnsafeMutablePointer<UInt8>.allocate(capacity : 4 )
                array_int.initialize(from: [0, 0, 0, 0])
                // var new_variable:Int
                response.value?.copyBytes(to: array_int, count: 1)
                print( "array is", array_int[0])
                if( array_int[0] == 6 )
                {
                    result = 1
                    self.timer.invalidate()
                    self.makeConfirmRequest()
                    self.change_ui(val:result)
                }
                else if( array_int[0] == 5 )
                {
                    result = 0
                    self.timer.invalidate()
                    self.makeConfirmRequest()
                    self.change_ui(val:result)
                }
        }
    }
    
    func makePostRequest() {
         print("post req")
        let parameters: Parameters = [
            "ssid_name"  : DataManager.sharedInstance().homeAP,
            "ssid_passphrase":DataManager.sharedInstance().homeAPpwd
        ]
        
     
        let req = self.afManager?.request("https://www.wiced.com/networkconfig", method: .post, parameters: parameters, encoding: JSONEncoding.default, headers: nil)
            .response {
                response in
                print( "status code is: ", response.response?.statusCode)
                if let status = response.response?.statusCode {
                    switch(status)
                    {
                    case 200:
                        self.scheduledTimerWithTimeInterval()
                    default:
                        print("Error while setting network configuration")
                    }
                }
        }
        
        print("Status : ", (req?.response?.statusCode) ?? "Default status")
        //self.makGetRequest()
        
    }
    
    func checkstatus() {
        NSLog("Check status...")
        self.makeGetRequest()
    }
    
    func scheduledTimerWithTimeInterval() {
        // Scheduling timer to Call the function **Countdown** with the interval of 1 seconds
        timer = Timer.scheduledTimer(timeInterval: 2, target: self, selector: #selector(self.checkstatus), userInfo: nil, repeats: true)
    }
    
  
    
    func configureAlamoFireSSLPinning() {
        let pathToCert = Bundle.main.path(forResource: "certificate", ofType: "der")
        let localCertificate:NSData = NSData(contentsOfFile: pathToCert!)!
        
        print(localCertificate)
        
        self.serverTrustPolicy = ServerTrustPolicy.pinCertificates(
            certificates: [SecCertificateCreateWithData(nil, localCertificate)!],
            validateCertificateChain: true,
            validateHost: true
        )
        
        self.serverTrustPolicies = [
            "wiced.com": self.serverTrustPolicy!
        ]
        
        
        self.afManager = SessionManager(
            configuration: URLSessionConfiguration.default,
            serverTrustPolicyManager: ServerTrustPolicyManager(policies: self.serverTrustPolicies)
        )
    }

    func bypassURLAuthentication() {
        
        self.afManager?.delegate.sessionDidReceiveChallenge = { session, challenge in
            var disposition: URLSession.AuthChallengeDisposition = .performDefaultHandling
            var credential: URLCredential?
            if challenge.protectionSpace.authenticationMethod == NSURLAuthenticationMethodServerTrust {
                
               let verified = (challenge.protectionSpace.host.compare("www.wiced.com",
                                                                     options: [.caseInsensitive], range: nil, locale: nil) == .orderedSame)
                if (verified == true)
                {
                    disposition = URLSession.AuthChallengeDisposition.useCredential
                    credential = URLCredential(trust: challenge.protectionSpace.serverTrust!)
                    print(" verified is true")
                } else {
                    print(" verified is false")
                }
               
            } else {
                if challenge.previousFailureCount > 0 {
                    disposition = .cancelAuthenticationChallenge
                } else {
                    credential = self.afManager?.session.configuration.urlCredentialStorage?.defaultCredential(for: challenge.protectionSpace)
                    if credential != nil {
                        disposition = .useCredential
                    }
                }
            }
            return (disposition, credential)
        }
    }
    
    
}

