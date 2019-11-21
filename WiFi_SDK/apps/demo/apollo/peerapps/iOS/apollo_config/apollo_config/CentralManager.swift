//
//  CentralManager.swift
//  apollo_config

/*
 * $ Copyright Broadcom Corporation $
 */

import Foundation
import CoreBluetooth

var thisCentralManager : CentralManager?

protocol ConnectPeripheralProtocol {
    var serviceUUIDString:String {get}
    var peripheralUUIDString:String {get}
    func didConnectPeripheral(cbPeripheral:CBPeripheral!)
    func didDisconnectPeripheral(cbPeripheral:CBPeripheral!, error:NSError!, userClickedCancel:Bool)
    func didRestorePeripheral(peripheral:Peripheral)
    func bluetoothBecomeAvailable()
    func bluetoothBecomeUnavailable()
}

public class CentralManager : NSObject, CBCentralManagerDelegate {
    
    var connectPeripheralDelegate : ConnectPeripheralProtocol!
    
    private var cbCentralManager : CBCentralManager!
    private let centralQueue = dispatch_queue_create("me.xuyuan.ble.central.main", DISPATCH_QUEUE_SERIAL)
    private var isScanning = false
    private var userClickedCancel = false
    private var afterPeripheralDiscovered : ((cbPeripheral:CBPeripheral, advertisementData:NSDictionary, RSSI:NSNumber)->())?

    // MARK: Singleton
    public class func sharedInstance() -> CentralManager {
        if thisCentralManager == nil {
            thisCentralManager = CentralManager()
        }
        return thisCentralManager!
    }
    
    private override init() {
        Logger.debug("CentralManager#init")
        super.init()
        self.cbCentralManager = CBCentralManager(delegate:self, queue:self.centralQueue, options:[CBCentralManagerOptionRestoreIdentifierKey:"mainCentralManagerIdentifier"])
    }
    
    // MARK: Public
    // scanning
    public func startScanning(afterPeripheralDiscovered:(cbPeripheral:CBPeripheral, advertisementData:NSDictionary, RSSI:NSNumber)->(), allowDuplicatesKey:Bool) {
        self.startScanningForServiceUUIDs([CBUUID(string: connectPeripheralDelegate.serviceUUIDString)], afterPeripheralDiscovered: afterPeripheralDiscovered, allowDuplicatesKey: allowDuplicatesKey)
    }
    
    public func startScanningForServiceUUIDs(uuids:[CBUUID]!, afterPeripheralDiscovered:(cbPeripheral:CBPeripheral, advertisementData:NSDictionary, RSSI:NSNumber)->(), allowDuplicatesKey:Bool) {
        if (!self.isScanning && cbCentralManager.state == CBCentralManagerState.PoweredOn) {
            Logger.debug("CentralManager#startScanningForServiceUUIDs: \(uuids) allowDuplicatesKey: \(allowDuplicatesKey)")
            self.isScanning = true
            self.afterPeripheralDiscovered = afterPeripheralDiscovered
            self.cbCentralManager.scanForPeripheralsWithServices(uuids,options: nil)
            
            //self.cbCentralManager.scanForPeripheralsWithServices(nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey : NSNumber(bool: true)])
        }
    }
    
    public func stopScanning() {
        if (self.isScanning) {
            Logger.debug("CentralManager#stopScanning")
            self.isScanning = false
            self.cbCentralManager.stopScan()
        }
    }
    
    // connection
    public func connectPeripheral(peripheral:Peripheral) {
        Logger.debug("CentralManager#connectPeripheral")
        
        self.cbCentralManager.connectPeripheral(peripheral.cbPeripheral, options : nil)
    }
    
    public func cancelPeripheralConnection(peripheral:Peripheral, userClickedCancel:Bool) {
        Logger.debug("CentralManager#cancelPeripheralConnection")
        self.cbCentralManager.cancelPeripheralConnection(peripheral.cbPeripheral)
        self.userClickedCancel = userClickedCancel
        if (userClickedCancel) {
            let userDefaults = NSUserDefaults.standardUserDefaults()
            userDefaults.setObject(nil, forKey: self.connectPeripheralDelegate.peripheralUUIDString)
            userDefaults.synchronize()
        }
    }

    // MARK: CBCentralManagerDelegate
    public func centralManagerDidUpdateState(central:CBCentralManager) {
        var statusText:String
        
        switch central.state {
        case CBCentralManagerState.PoweredOn:
            statusText = "Bluetooth powered on."
            connectPeripheralDelegate.bluetoothBecomeAvailable()
            //let userDefaults = NSUserDefaults.standardUserDefaults()
            //let peripheralUUID = userDefaults.stringForKey(STORED_PERIPHERAL_IDENTIFIER)
            //let peripheralUUID = STORED_PERIPHERAL_IDENTIFIER
            
                Logger.debug("CentralManager#retrievePeripheralsWithIdentifiers \(self.connectPeripheralDelegate.serviceUUIDString)")
                Utils.sendNotification("CentralManager#retrievePeripheralsWithIdentifiers \(self.connectPeripheralDelegate.serviceUUIDString)", soundName: "")
            print(self.connectPeripheralDelegate.peripheralUUIDString )
                for p:AnyObject in central.retrievePeripheralsWithIdentifiers([NSUUID(UUIDBytes: self.connectPeripheralDelegate.peripheralUUIDString)]) {
                   if p is CBPeripheral {
                        peripheralFound(p as! CBPeripheral)
                        return
                    }
                }
                Logger.debug("CentralManager#retrieveConnectedPeripheralsWithServices")
                for p:AnyObject in central.retrieveConnectedPeripheralsWithServices([CBUUID(string: self.connectPeripheralDelegate.serviceUUIDString)]) {
                    if p is CBPeripheral {
                        peripheralFound(p as! CBPeripheral)
                        return
                    }
                }
           
        case CBCentralManagerState.PoweredOff:
            statusText = "Bluetooth powered off."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        case CBCentralManagerState.Unsupported:
            statusText = "Bluetooth low energy hardware not supported."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        case CBCentralManagerState.Unauthorized:
            statusText = "Bluetooth unauthorized state."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        case CBCentralManagerState.Unknown:
            statusText = "Bluetooth unknown state."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        default:
            statusText = "Bluetooth unknown state."
            connectPeripheralDelegate.bluetoothBecomeUnavailable()
        }
        
        Logger.debug("CentralManager#centralManagerDidUpdateState: \(statusText)")
    }
    

    public func centralManager(central: CBCentralManager, didDiscoverPeripheral cbPeripheral: CBPeripheral, advertisementData: [String : AnyObject], RSSI: NSNumber) {
        
        
        print(cbPeripheral.name)
        
        print("identifier value: \(cbPeripheral.identifier)")
        
        Logger.debug("CentralManager#didDiscoverPeripheral \(cbPeripheral.name)")
        if let afterPeripheralDiscovered = self.afterPeripheralDiscovered {
            afterPeripheralDiscovered(cbPeripheral:cbPeripheral, advertisementData:advertisementData, RSSI:RSSI)
        }
    }
    
    public func centralManager(_:CBCentralManager, didConnectPeripheral peripheral:CBPeripheral) {
        Logger.debug("CentralManager#didConnectPeripheral")
        
        let userDefaults = NSUserDefaults.standardUserDefaults()
        userDefaults.setObject(peripheral.identifier.UUIDString as String, forKey: self.connectPeripheralDelegate.peripheralUUIDString)
        userDefaults.synchronize()
        
        if let connectPeripheralDelegate = self.connectPeripheralDelegate {
            connectPeripheralDelegate.didConnectPeripheral(peripheral)
        }
    }
    
    public func centralManager(_:CBCentralManager, didFailToConnectPeripheral peripheral:CBPeripheral, error:NSError?) {
        Logger.debug("CentralManager#didFailToConnectPeripheral")
    }
    
    public func centralManager(_:CBCentralManager, didDisconnectPeripheral peripheral:CBPeripheral, error:NSError?) {
        Logger.debug("CentralManager#didDisconnectPeripheral")
        if let connectPeripheralDelegate = self.connectPeripheralDelegate {
            connectPeripheralDelegate.didDisconnectPeripheral(peripheral, error: error, userClickedCancel: userClickedCancel)
        }
        userClickedCancel = false
    }

    public func centralManager(_:CBCentralManager!, didRetrieveConnectedPeripherals peripherals:[AnyObject]!) {
        Logger.debug("CentralManager#didRetrieveConnectedPeripherals")
    }
    
    public func centralManager(_:CBCentralManager!, didRetrievePeripherals peripherals:[AnyObject]!) {
        Logger.debug("CentralManager#didRetrievePeripherals")
    }
    
    public func centralManager(central: CBCentralManager , willRestoreState dict: [String : AnyObject]) {
        if let _:[CBPeripheral] = dict[CBCentralManagerRestoredStatePeripheralsKey] as! [CBPeripheral]! {
            Logger.debug("CentralManager#willRestoreState")
        }
    }

    // MARK: Private
    private func peripheralFound(cbPeripheral: CBPeripheral) {
        Logger.debug("CentralManager#peripheralFound \(cbPeripheral.name)")
        let peripheral:Peripheral = Peripheral(cbPeripheral: cbPeripheral, advertisements:[:], rssi:0)
        self.connectPeripheralDelegate.didRestorePeripheral(peripheral)
    }

}