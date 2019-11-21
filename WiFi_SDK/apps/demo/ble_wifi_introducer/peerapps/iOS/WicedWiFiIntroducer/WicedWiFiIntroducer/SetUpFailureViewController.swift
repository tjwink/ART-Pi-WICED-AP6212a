//
//  SetUpFailureViewController.swift
//  WicedWiFiIntroducer
//
//  Created by bluth on 29/03/16.
//  Copyright © 2016 bluth. All rights reserved.
//

import UIKit


protocol SetUpFailureViewControllerDelegate {
    func tryAgain()
}

class SetUpFailureViewController: UIViewController {
    @IBOutlet weak var deviceName: UILabel!

    @IBOutlet weak var WifiNetworkText: UILabel!
    var delegate: SetUpFailureViewControllerDelegate?

    override func viewDidLoad() {

        super.viewDidLoad()
        var devName = "BRCM Wiced Device"
        for peripheral:Peripheral in DataManager.sharedInstance().discoveredPeripherals.values {
            devName = peripheral.name
        }

        if devName != "BRCM Wiced Device" {
            deviceName.text = "\u{2022} \(devName)"
        }
        let ssidName1 = SSIDNetwork()

        if let temp = ssidName1.getSSID() {

            WifiNetworkText.text = "\u{2022} \(temp)"
            
        }
    }

    @IBAction func OnTryAgainButtonClicked(_ sender: AnyObject) {
        notifyCallingViewController();
    }

    func notifyCallingViewController() {
        print("notifyCallingViewController tryAgain")
        self.delegate?.tryAgain()
    }
}
