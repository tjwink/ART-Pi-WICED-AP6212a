//
//  ViewController.swift
//  WicedWiFiIntroducer
//
//  Created by bluth on 22/03/16.
//  Copyright © 2016 bluth. All rights reserved.
//

import UIKit

class ViewController: UIViewController, ScanViewControllerDelegate, WirelessSetupViewControllerDelegate, SetUpFailureViewControllerDelegate {

    @IBOutlet weak var viewcontainer: UIView!
    weak var currentViewController: UIViewController?

    override func viewDidLoad() {
        let tempController: ScanningConfigureViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentA") as! ScanningConfigureViewController

        tempController.data = true
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
            
        } else if  value == 1{

            let newViewController:WirelessSetupViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentB") as! WirelessSetupViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            newViewController.delegate = self;
            self.currentViewController = newViewController

        } else if value == 2 {

            let newViewController:SetUpSuccessViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentC") as! SetUpSuccessViewController
            self.cycleFromViewController(self.currentViewController!, toViewController: newViewController)
            self.currentViewController = newViewController

        } else if value == 3 {

            let newViewController:SetUpFailureViewController = self.storyboard?.instantiateViewController(withIdentifier: "ComponentD") as! SetUpFailureViewController
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

        print("yeeeehhh .. I reached scanningComplete")
        showComponent(1)

    }
    func setUpComplete(_ data: Bool) {

        print("yeeeehhh .. I reached setUpComplete")
        DispatchQueue.main.async(execute: {
            if data {
                self.showComponent(2)
            } else {
                self.showComponent(3)
            }
        })

        
    }

    func tryAgain() {
        print("yeeeehhh .. I reached tryAgain")
        DispatchQueue.main.async(execute: {
            self.showComponent(1)
        })

    }

}

