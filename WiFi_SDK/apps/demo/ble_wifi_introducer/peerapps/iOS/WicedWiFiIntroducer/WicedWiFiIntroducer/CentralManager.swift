//
//  CentralManager.swift
//  testing
//
//  Copyright © 2015 broadcom. All rights reserved.
//

import Foundation
import CoreBluetooth

var thisCentralManager : CentralManager?

protocol ConnectPeripheralProtocol {
    var serviceUUIDString:String {get}
    var peripheralUUIDString:String {get}
    func didConnectPeripheral(_ cbPeripheral:CBPeripheral!)
    func didDisconnectPeripheral(_ cbPeripheral:CBPeripheral!, error:NSError!, userClickedCancel:Bool)
    func didRestorePeripheral(_ peripheral:Peripheral)
    func bluetoothBecomeAvailable()
    func bluetoothBecomeUnavailable()
}

open class CentralManager : NSObject, CBCentralManagerDelegate {
    
    var connectPeripheralDelegate : ConnectPeripheralProtocol!
    
    fileprivate var cbCentralManager : CBCentralManager!
    fileprivate let centralQueue = DispatchQueue(label: "me.xuyuan.ble.central.main", attributes: [])
    open var isScanning = false
    fileprivate var userClickedCancel = false
    fileprivate var afterPeripheralDiscovered : ((_ cbPeripheral:CBPeripheral, _ advertisementData:NSDictionary, _ RSSI:NSNumber)->())?

    // MARK: Singleton
    open class func sharedInstance() -> CentralManager {
        if thisCentralManager == nil {
            thisCentralManager = CentralManager()
        }
        return thisCentralManager!
    }
    
    fileprivate override init() {
        Logger.debug("CentralManager#init" as AnyObject)
        super.init()
        self.cbCentralManager = CBCentralManager(delegate:self, queue:self.centralQueue, options:[CBCentralManagerOptionRestoreIdentifierKey:"mainCentralManagerIdentifier"])
    }
    
    // MARK: Public
    // scanning
    open func startScanning(_ afterPeripheralDiscovered:@escaping (_ cbPeripheral:CBPeripheral, _ advertisementData:NSDictionary, _ RSSI:NSNumber)->(), allowDuplicatesKey:Bool) {
        self.startScanningForServiceUUIDs([CBUUID(string: connectPeripheralDelegate.serviceUUIDString)], afterPeripheralDiscovered: afterPeripheralDiscovered, allowDuplicatesKey: allowDuplicatesKey)
    }
    
    open func startScanningForServiceUUIDs(_ uuids:[CBUUID]!, afterPeripheralDiscovered:@escaping (_ cbPeripheral:CBPeripheral, _ advertisementData:NSDictionary, _ RSSI:NSNumber)->(), allowDuplicatesKey:Bool) {
        if (!self.isScanning && cbCentralManager.state == .poweredOn) {
            Logger.debug("CentralManager#startScanningForServiceUUIDs: \(uuids) allowDuplicatesKey: \(allowDuplicatesKey)" as AnyObject)
            self.isScanning = true
            self.afterPeripheralDiscovered = afterPeripheralDiscovered
            self.cbCentralManager.scanForPeripherals(withServices: uuids,options: [CBCentralManagerScanOptionAllowDuplicatesKey : allowDuplicatesKey])
            
            //self.cbCentralManager.scanForPeripheralsWithServices(nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey : NSNumber(bool: true)])
        }
    }
    
    open func stopScanning() {
        //if (self.isScanning) {
            Logger.debug("CentralManager#stopScanning" as AnyObject)
            self.isScanning = false
            self.cbCentralManager.stopScan()
        //}
    }
    
    // connection
    open func connectPeripheral(_ peripheral:Peripheral) {
        Logger.debug("CentralManager#connectPeripheral" as AnyObject)
        
        self.cbCentralManager.connect(peripheral.cbPeripheral, options : nil)
    }
    
    open func cancelPeripheralConnection(_ peripheral:Peripheral, userClickedCancel:Bool) {
        Logger.debug("CentralManager#cancelPeripheralConnection" as AnyObject)
        self.cbCentralManager.cancelPeripheralConnection(peripheral.cbPeripheral)
        self.userClickedCancel = userClickedCancel
        if (userClickedCancel) {
            let userDefaults = UserDefaults.standard
            userDefaults.set(nil, forKey: self.connectPeripheralDelegate.peripheralUUIDString)
            userDefaults.synchronize()
        }
    }

    // MARK: CBCentralManagerDelegate
    open func centralManagerDidUpdateState(_ central:CBCentralManager) {
        var statusText:String
        
        switch central.state {
        case .poweredOn:
            statusText = "Bluetooth powered on."
            connectPeripheralDelegate.bluetoothBecomeAvailable()
            //let userDefaults = NSUserDefaults.standardUserDefaults()
            //let peripheralUUID = userDefaults.stringForKey(STORED_PERIPHERAL_IDENTIFIER)
            //let peripheralUUID = STORED_PERIPHERAL_IDENTIFIER
            
                //Logger.debug("CentralManager#retrievePeripheralsWithIdentifiers \(self.connectPeripheralDelegate.serviceUUIDString)")
                //Utils.sendNotification("CentralManager#retrievePeripheralsWithIdentifiers \(self.connectPeripheralDelegate.serviceUUIDString)", soundName: "")
            //print(self.connectPeripheralDelegate.peripheralUUIDString )
                //for p:AnyObject in central.retrievePeripheralsWithIdentifiers([NSUUID(UUIDBytes: self.connectPeripheralDelegate.peripheralUUIDString)]) {
                   //if p is CBPeripheral {
                        //peripheralFound(p as! CBPeripheral)
                        //return
                    //}
               // }
                //Logger.debug("CentralManager#retrieveConnectedPeripheralsWithServices")
                //for p:AnyObject in central.retrieveConnectedPeripheralsWithServices([CBUUID(string: self.connectPeripheralDelegate.serviceUUIDString)]) {
                    //if p is CBPeripheral {
                       // peripheralFound(p as! CBPeripheral)
                       // return
                    //}
               // }
           
        case .poweredOff:
            statusText = "Bluetooth powered off."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        case .unsupported:
            statusText = "Bluetooth low energy hardware not supported."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        case .unauthorized:
            statusText = "Bluetooth unauthorized state."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        case .unknown:
            statusText = "Bluetooth unknown state."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        default:
            statusText = "Bluetooth unknown state."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        }
        
        Logger.debug("CentralManager#centralManagerDidUpdateState: \(statusText)" as AnyObject)
    }
    

    open func centralManager(_ central: CBCentralManager, didDiscover cbPeripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        
        
       // if cbPeripheral.name == "WiFiInt" {
            
            //Logger.debug("CentralManager#didDiscoverPeripheral \(cbPeripheral.name)")
            if let afterPeripheralDiscovered = self.afterPeripheralDiscovered {
                afterPeripheralDiscovered(cbPeripheral, advertisementData as NSDictionary, RSSI)
            }
            
        //}
        
        
    }
    
    open func centralManager(_:CBCentralManager, didConnect peripheral:CBPeripheral) {
        Logger.debug("CentralManager#didConnectPeripheral" as AnyObject)
        
        let userDefaults = UserDefaults.standard
        userDefaults.set(peripheral.identifier.uuidString as String, forKey: self.connectPeripheralDelegate.peripheralUUIDString)
        userDefaults.synchronize()
        
        if let connectPeripheralDelegate = self.connectPeripheralDelegate {
            connectPeripheralDelegate.didConnectPeripheral(peripheral)
        }
    }
    
    open func centralManager(_:CBCentralManager, didFailToConnect peripheral:CBPeripheral, error:Error?) {
        Logger.debug("CentralManager#didFailToConnectPeripheral" as AnyObject)
    }
    
    open func centralManager(_:CBCentralManager, didDisconnectPeripheral peripheral:CBPeripheral, error:Error?) {
        Logger.debug("CentralManager#didDisconnectPeripheral" as AnyObject)
        if let connectPeripheralDelegate = self.connectPeripheralDelegate {
            connectPeripheralDelegate.didDisconnectPeripheral(peripheral, error: error as NSError!, userClickedCancel: userClickedCancel)
        }
        userClickedCancel = false
    }

    open func centralManager(_:CBCentralManager!, didRetrieveConnectedPeripherals peripherals:[AnyObject]!) {
        Logger.debug("CentralManager#didRetrieveConnectedPeripherals" as AnyObject)
    }
    
    open func centralManager(_:CBCentralManager!, didRetrievePeripherals peripherals:[AnyObject]!) {
        Logger.debug("CentralManager#didRetrievePeripherals" as AnyObject)
    }
    
    open func centralManager(_ central: CBCentralManager , willRestoreState dict: [String : Any]) {
        if let _:[CBPeripheral] = dict[CBCentralManagerRestoredStatePeripheralsKey] as! [CBPeripheral]! {
            Logger.debug("CentralManager#willRestoreState" as AnyObject)
        }
    }

    // MARK: Private
    fileprivate func peripheralFound(_ cbPeripheral: CBPeripheral) {
        Logger.debug("CentralManager#peripheralFound \(cbPeripheral.name)" as AnyObject)
        let peripheral:Peripheral = Peripheral(cbPeripheral: cbPeripheral, advertisements:[:], rssi:0)
        self.connectPeripheralDelegate.didRestorePeripheral(peripheral)
    }

}