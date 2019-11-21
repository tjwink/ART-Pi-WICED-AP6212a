//
//  SetUpSuccessViewController.swift
//  WicedWiFiIntroducer
//
//  Created by bluth on 29/03/16.
//  Copyright © 2016 bluth. All rights reserved.
//

import UIKit

class SetUpSuccessViewController: UIViewController {


    @IBOutlet weak var deviceName: UILabel!

    @IBOutlet weak var WifiNetworkText: UILabel!

    override func viewDidLoad() {

        super.viewDidLoad()

        let ssidName1 = SSIDNetwork()

        var devName = "BRCM Wiced Device"
        for peripheral:Peripheral in DataManager.sharedInstance().discoveredPeripherals.values {
            devName = peripheral.name
        }
        if devName != "BRCM Wiced Device" {
            deviceName.text = "\u{2022} \(devName)"
        }
        if let temp = ssidName1.getSSID() {

            WifiNetworkText.text = "\u{2022} \(temp)"

        }
    }

    @IBAction func OnDoneButtonClick(_ sender: AnyObject) {
        UIControl().sendAction(#selector(URLSessionTask.suspend), to: UIApplication.shared, for: nil)
    }

}
