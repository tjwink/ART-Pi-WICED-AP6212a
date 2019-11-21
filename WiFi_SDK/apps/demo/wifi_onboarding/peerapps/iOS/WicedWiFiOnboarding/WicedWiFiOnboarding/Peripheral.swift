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

import Foundation
import CoreBluetooth

// Service UUID
let WiFiIntroducerConfigNWMConfigServiceUUID    = CBUUID(string: "1B7E8251-2877-41C3-B46E-CF057C562023")
// Characteristic UUIDs
let WiFiIntroducerConfigNWModeUUID              = CBUUID(string: "5B1C19EF-9DD9-4BC4-B848-CFCFDE16B861")
let WiFiIntroducerConfigNWSSIDUUID              = CBUUID(string: "ACA0EF7C-EEAA-48AD-9508-19A6CEF6B356")
let WiFiIntroducerConfigNWSecurityUUID          = CBUUID(string: "CAC2ABA4-EDBB-4C4A-BBAF-0A84A5CD93A1")
let WiFiIntroducerConfigNWPassphraseUUID        = CBUUID(string: "40B7DE33-93E4-4C8B-A876-D833B415A6CE")
let WiFiIntroducerConfigNWChannelUUID           = CBUUID(string: "87F3E72E-94A9-4453-BA85-DCF95490F0EB")
let WiFiIntroducerConfigNWBandUUID              = CBUUID(string: "F000AA22-0451-4000-B000-000000000000")
let WiFiIntroducerConfigSpeakerNameUUID         = CBUUID(string: "FC352C5B-536D-4E4A-B97E-4B3F0705D11D")
let WiFiIntroducerConfigSpeakerChannelUUID      = CBUUID(string: "DA6808BD-F08F-41F7-9325-008948779759")
let WiFiIntroducerConfigSpeakerOutputPortUUID   = CBUUID(string: "F000AA41-0451-4000-B000-000000000000")
let WiFiIntroducerConfigSpeakerOutputVolumeUUID = CBUUID(string: "98749EAB-2DCC-4DA4-B533-5016F7A06B2C")
let WiFiIntroducerConfigSourceInputUUID         = CBUUID(string: "1CD734D3-5592-4FA3-B0AB-826473B2F8B0")
let WiFiIntroducerConfigSourceLoopbackUUID      = CBUUID(string: "F000AA52-0451-4000-B000-000000000000")
let WiFiIntroducerConfigSourceInputPortUUID     = CBUUID(string: "F000AA51-0451-4000-B000-000000000000")
let WiFiIntroducerConfigSourceInputVolumeUUID   = CBUUID(string: "04C77CE6-D0AB-4B86-BC10-D33625E059F6")
let WiFiIntroducerConfigCharNotifyValueUUID     = CBUUID(string: "8AC32D3f-5CB9-4D44-BEC2-EE689169F626")

protocol ReadPeripheralProtocol {
    var serviceUUIDString:String {get}
    var characteristicUUIDString:String {get}
    func didDiscoverCharacteristicsofPeripheral(_ cbservice : CBService!)
    func didUpdateValueForCharacteristic(_ cbPeripheral: CBPeripheral!, characteristic:CBCharacteristic!, error:NSError!)
    func didWriteValueForCharacteristic(_ cbPeripheral: CBPeripheral!, characteristic:CBCharacteristic!, error:NSError!)
}

open class Peripheral : NSObject, CBPeripheralDelegate {
    var readPeripheralDelegate:ReadPeripheralProtocol!
    
    // INTERNAL
    internal var cbPeripheral    : CBPeripheral!
    
    // MARK: Public
    open var advertisements  : Dictionary<NSObject, AnyObject>!
    open var rssi            : Int!
    
    fileprivate var _name : String?
    open var name : String {
        get{
            // iOS does not advertise peripheral name in background
            // and even the peripheral is in foreground, the central might still use peripheral's old cached name
            // So only use peripheral's name when explicit name is unavialable
            if(_name == nil) {
                if let name = cbPeripheral.name {
                    return name
                } else {
                    return "Unknown"
                }
            } else {
                return _name!
            }
        }
        set{
            _name = newValue
        }
    }
    
    open var installationId : String?
    
    open var isNearby = false
    
    open var hasBeenConnected = false
    
    open var characteristicWriteCount = 0
    
    open var isSender = false
    
    open var senderName = ""
    open var speakerName = ""
    
    open var state : CBPeripheralState {
        return self.cbPeripheral.state
    }
    
    open var identifier : UUID! {
        return self.cbPeripheral.identifier
    }
    
    public init(cbPeripheral:CBPeripheral, advertisements:Dictionary<NSObject, AnyObject>, rssi:Int) {
        super.init()
        self.cbPeripheral = cbPeripheral
        // Fix bug: cbPeripheral.delegate will point to wrong instance because select peripheral screen refresh too fast
        // Move to Peripheral#discoverServices
        // self.cbPeripheral.delegate = self
        self.advertisements = advertisements
        self.rssi = rssi
    }
    
    func discoverServices(_ serviceUUIDs: [CBUUID]!, delegate: ReadPeripheralProtocol!) {
        self.cbPeripheral.delegate = self
        self.readPeripheralDelegate = delegate
        self.cbPeripheral.discoverServices(serviceUUIDs)
    }
    
    // MARK: CBPeripheralDelegate
    // peripheral
    open func peripheralDidUpdateName(_:CBPeripheral) {
        Logger.debug("Peripheral#peripheralDidUpdateName" as AnyObject)
    }
    
    @nonobjc open func peripheral(_:CBPeripheral!, didModifyService invalidatedServices:[AnyObject]!) {
        if let delegate = self.readPeripheralDelegate {
            for service:CBService in invalidatedServices as! [CBService]! {
                if (service.uuid.uuidString == delegate.serviceUUIDString) {
                    Logger.debug("Peripheral#didModifyServices \(service)" as AnyObject)
                    CentralManager.sharedInstance().cancelPeripheralConnection(self, userClickedCancel: false)
                }
            }
        }
    }
    
    // services
    open func peripheral(_ peripheral:CBPeripheral, didDiscoverServices error:Error?) {
        Logger.debug("Peripheral#didDiscoverServices: \(self.name) error: \(error)" as AnyObject)
        if (error == nil) {
            if let _:ReadPeripheralProtocol = self.readPeripheralDelegate {
                for service:CBService in peripheral.services as [CBService]! {
                    peripheral.discoverCharacteristics(nil, for: service)
                }
            }
        }
    }
    
    open func peripheral(_:CBPeripheral, didDiscoverIncludedServicesFor service:CBService, error:Error?) {
        Logger.debug("Peripheral#didDiscoverIncludedServicesForService: \(self.name) error: \(error)" as AnyObject)
    }
    
    // characteristics
    open func peripheral(_:CBPeripheral, didDiscoverCharacteristicsFor service:CBService, error:Error?) {
        Logger.debug("Peripheral#didDiscoverCharacteristicsForService: \(self.name) error: \(error)" as AnyObject)
        
        if (error == nil) {
            // Logger.debug("Peripheral#didUpdateValueForCharacteristic")
            if let delegate:ReadPeripheralProtocol = self.readPeripheralDelegate {
                
                delegate.didDiscoverCharacteristicsofPeripheral( service )
            }
        }
    }
    
    open func peripheral(_:CBPeripheral, didUpdateNotificationStateFor characteristic:CBCharacteristic, error:Error?) {
        Logger.debug("Peripheral#didUpdateNotificationStateForCharacteristic error: \(error)" as AnyObject)
    }
    
    open func peripheral(_ peripheral:CBPeripheral, didUpdateValueFor characteristic:CBCharacteristic, error:Error?) {
        // Logger.debug("Peripheral#didUpdateValueForCharacteristic")
        if let delegate:ReadPeripheralProtocol = self.readPeripheralDelegate {
            delegate.didUpdateValueForCharacteristic(peripheral, characteristic: characteristic, error: error as NSError!)
        }
    }
    
    open func peripheral(_ peripheral:CBPeripheral, didWriteValueFor characteristic:CBCharacteristic, error: Error?) {
        Logger.debug("Peripheral#didWriteValueForCharacteristic error: \(error)" as AnyObject)

        if let delegate:ReadPeripheralProtocol = self.readPeripheralDelegate {
            delegate.didWriteValueForCharacteristic(peripheral, characteristic: characteristic, error: error as NSError!)
        }
    }
    
    // descriptors
    open func peripheral(_:CBPeripheral, didDiscoverDescriptorsFor characteristic:CBCharacteristic, error:Error?) {
        Logger.debug("Peripheral#didDiscoverDescriptorsForCharacteristic error: \(error)" as AnyObject)
    }
    
    open func peripheral(_:CBPeripheral, didUpdateValueFor descriptor:CBDescriptor, error:Error?) {
        Logger.debug("Peripheral#didUpdateValueForDescriptor error: \(error)" as AnyObject)
    }
    
    open func peripheral(_:CBPeripheral, didWriteValueFor descriptor:CBDescriptor, error:Error?) {
        Logger.debug("Peripheral#didWriteValueForDescriptor error: \(error)" as AnyObject)
    }
    
}
