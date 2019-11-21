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

class ViewController: UIViewController, ScanViewControllerDelegate, WirelessSetupViewControllerDelegate, SetUpFailureViewControllerDelegate, SetDeviceViewControllerDelegate, SetUpSuccessViewControllerDelegate {

    @IBOutlet weak var viewcontainer: UIView!
    weak var currentViewController: UIViewController?

    override func viewDidLoad() {
 //       let tempController: ScanningConfigureViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentA") as! ScanningConfigureViewController
        
        let tempController: SetDeviceViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentE") as! SetDeviceViewController
        
       // tempController.data = true
        tempController.delegate = self;
        self.currentViewController = tempController

        self.addChildViewController(self.currentViewController!)

        self.currentViewController?.view.frame = CGRect(x: 0,y: 0,width: viewcontainer.frame.size.width, height: viewcontainer.frame.size.height)


        self.addSubview(self.currentViewController!.view, toView: self.viewcontainer)
        super.viewDidLoad()
    }

    func buttonAction(_ sender:UIButton!)
    {
        print("Button tapped")
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    func addSubview(_ subView:UIView, toView parentView:UIView) {
        parentView.addSubview(subView)

        var viewBindingsDict = [String: AnyObject]()
        viewBindingsDict["subView"] = subView
        parentView.addConstraints(NSLayoutConstraint.constraints(withVisualFormat: "H:|[subView]|",
            options: [], metrics: nil, views: viewBindingsDict))
        parentView.addConstraints(NSLayoutConstraint.constraints(withVisualFormat: "V:|[subView]|",
            options: [], metrics: nil, views: viewBindingsDict))
    }

     func showComponent(_ value: NSInteger) {
        if value == 0 {

            let newViewController:ScanningConfigureViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentA") as! ScanningConfigureViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            newViewController.data = true
            newViewController.delegate = self;
            self.currentViewController = newViewController
            
        } else if  value == 1 {

            let newViewController:WirelessSetupViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentB") as! WirelessSetupViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            newViewController.delegate = self;
            self.currentViewController = newViewController

        } else if value == 2 {

            let newViewController:SetUpSuccessViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentC") as! SetUpSuccessViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            self.currentViewController = newViewController
            newViewController.delegate = self;

        } else if value == 3 {

            let newViewController:SetUpFailureViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentD") as! SetUpFailureViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            newViewController.delegate = self;
            self.currentViewController = newViewController

        } else if value == 4 {
            let newViewController:SetDeviceViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentE") as! SetDeviceViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            newViewController.delegate = self;
            self.currentViewController = newViewController
        }
    }

    func cycleFromViewController(_ oldViewController: UIViewController, toViewController newViewController: UIViewController) {
        oldViewController.willMove(toParentViewController: nil)
        self.addChildViewController(newViewController)
        newViewController.view.frame = CGRect(x: 0,y: 0,width: viewcontainer.frame.size.width, height: viewcontainer.frame.size.height)
        self.addSubview(newViewController.view, toView:self.viewcontainer!)
        newViewController.view.alpha = 0
        newViewController.view.layoutIfNeeded()


        UIView.animate(withDuration: 2, animations: {
            newViewController.view.alpha = 1
            oldViewController.view.alpha = 0
            },
            completion: { finished in
               oldViewController.view.removeFromSuperview()
                oldViewController.removeFromParentViewController()
               newViewController.didMove(toParentViewController: self)

        })
    }

    func scanningComplete(_ data: Bool) {

        print("scanningComplete")
        showComponent(4)

    }
    func setUpComplete(_data data: Bool) {

        print("setUpComplete")
        DispatchQueue.main.async(execute: {
            if data {
                self.showComponent(2)
            } else {
                self.showComponent(3)
            }
        })

        
    }

    func tryAgain() {
        print("tryAgain")
        DispatchQueue.main.async(execute: {
            self.showComponent(1)
        })

    }
    
    func onDeviceAPConnected() {
        print("onDeviceAPConnected")
        self.showComponent(1)
        
    }
    
    func beginOnboarding() {
        print("beginOnBoarding")
        DispatchQueue.main.async(execute: {
            self.showComponent(1)
        })
        
    }

}

