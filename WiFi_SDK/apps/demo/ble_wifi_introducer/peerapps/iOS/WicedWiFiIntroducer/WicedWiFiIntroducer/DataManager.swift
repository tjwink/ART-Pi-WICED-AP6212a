import CoreBluetooth


var thisDataManager : DataManager?

class DataManager {
    
    var serviceUUIDString:String        = "1B7E8251-2877-41C3-B46E-CF057C562023"
    var peripheralUUIDString:String     = "7EC612F7-F388-0284-624C-BE76C314C6CD"
    var characteristicUUIDString:String = "B6251F0B-3869-4C0D-ACAB-D93F45187E6F"
    
    var discoveredPeripherals : Dictionary<CBPeripheral, Peripheral> = [:]
    var peripheralInProximity : Bool = false
    var timer = Timer()
    var writeCount = 0
    var firstPeripheralCount = 0;
    
    
    // MARK: Singleton
    open class func sharedInstance() -> DataManager {
        if thisDataManager == nil {
            thisDataManager = DataManager()
        }
        return thisDataManager!
    }

}