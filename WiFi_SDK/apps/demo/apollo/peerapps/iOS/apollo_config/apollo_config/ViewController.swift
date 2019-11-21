//
//  ViewController.swift
//  apollo_config

/*
 * $ Copyright Broadcom Corporation $
 */

import UIKit
import CoreBluetooth

class ViewController: UIViewController, UITableViewDelegate, UITableViewDataSource,
    ConnectPeripheralProtocol, ReadPeripheralProtocol, UITextFieldDelegate {

    // MARK: Properties

    @IBOutlet weak var tableView: UITableView!

    var data: DataManager? = nil
    var snapshot: UIView? = nil
    var sourceIndexPath: NSIndexPath? = nil
    var prevLocation: CGPoint? = nil
    var discoveredPeripherals : Dictionary<CBPeripheral, Peripheral> = [:]

    var serviceUUIDString:String        = "04574543-9CBA-4B22-BC78-55040A01C246"
    var characteristicUUIDString:String = "B6251F0B-3869-4C0D-ACAB-D93F45187E6F"
    var peripheralUUIDString:String     = "76B1FDBA-FE17-2A00-0000-000000000000"

    var peripheralsTableDataCopy = [cellData]()

    var senderName:String = ""
    var speakerName:String = ""


    var bluetoothOn = false

    override func viewDidLoad() {
        super.viewDidLoad()

        tableView.registerClass(UITableViewCell.self, forCellReuseIdentifier: "AudioConfigTableViewCell")
        // Do any additional setup after loading the view, typically from a nib.

        let longPress: UILongPressGestureRecognizer = UILongPressGestureRecognizer(target: self, action: "longPressGestureRecognized:")
        self.tableView.addGestureRecognizer(longPress)

        tableView.backgroundColor = UIColor.clearColor()

        if CentralManager.sharedInstance().connectPeripheralDelegate == nil {
            initBluetooth()
        }
        else {
            bluetoothBecomeAvailable()
        }

        // incrementally increase the wait time and then reset back to the minimum
        var sleepCount = 2
        while(discoveredPeripherals.count == 0) {
            if sleepCount != 10 {
                sleep(UInt32(sleepCount))
                sleepCount = sleepCount + 2
            }
            else {

                //let alertView = UIAlertController(title: "Alert", message: "Scanning couldn't find any WICED device. Kindly power it ON!", preferredStyle: .Alert)
                //alertView.addAction(UIAlertAction(title: "Ok", style: .Default, handler: nil))
                //presentViewController(alertView, animated: true, completion: nil)

                if bluetoothOn == false {
                    print("\nPlease turn ON bluetooth!\n")
                }
                else {
                    print("\nScanning couldn't find any WICED device. Kindly power it ON!\n")
                }

                sleepCount = 2
                continue
            }
        }

        //once all the BLE devices are detected then pass it to the data source
        data = DataManager(test : discoveredPeripherals)
    }

    deinit {

        data = nil
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    required init(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)!
    }

    @IBAction func clickBackButton(sender: AnyObject) {

        data = nil
    }

    override func viewWillDisappear(animated: Bool) {

        if data != nil {
            data = nil
        }

        stopScanning()
    }

    // to put all the cells' text field into Edit mode
    // so that user can enter sender/speaker names

    @IBAction func editBarButtonClick( sender: AnyObject) {

        var peripheral : Peripheral;

        // find a better way of doing this?

        peripheralsTableDataCopy = data!.valueofCells()

        if (sender.title == "Edit") {

            self.navigationItem.rightBarButtonItem?.title = "Done"

            for (var i = 0; i < self.peripheralsTableDataCopy.count; i++) {

                if !self.peripheralsTableDataCopy[i].isSection {

                    for var section = 0; section < self.tableView.numberOfSections; section++ {

                        for var row = 0; row < self.tableView.numberOfRowsInSection(section); row++ {

                            let cellPath = NSIndexPath(forRow:row, inSection: section)

                            // all these computations are to give a text name to the peripheralsTableDataCopy
                            // based on the ID

                            let cellID = self.data!.cellID(cellPath)

                            if cellID == "cellPeripheral" {

                                let cell:AudioConfigTableViewCell = self.tableView.cellForRowAtIndexPath(cellPath) as!
                                AudioConfigTableViewCell

                                if row == self.peripheralsTableDataCopy[i].trueIndex {

                                    cell.speakerNameTextField.enabled = true
                                    // Handle the text field’s user input through delegate callbacks.
                                    cell.speakerNameTextField.delegate = self

                                    // sender row becomes the first responder
                                    if self.peripheralsTableDataCopy[i].trueIndex == 1 {
                                        cell.speakerNameTextField.becomeFirstResponder()
                                    }

                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        else {

            self.navigationItem.rightBarButtonItem?.title = "Edit"

            for (var i = 0; i < self.peripheralsTableDataCopy.count; i++) {

                if !self.peripheralsTableDataCopy[i].isSection {

                    peripheral = self.peripheralsTableDataCopy[i].peripheralMeta!

                    for var section = 0; section < self.tableView.numberOfSections; section++ {

                        for var row = 0; row < self.tableView.numberOfRowsInSection(section); row++ {

                            let cellPath = NSIndexPath(forRow:row, inSection: section)

                            // all these computations are to give a text name to the peripheralsTableDataCopy
                            // based on the ID

                            let cellID = self.data!.cellID(cellPath)

                            if cellID == "cellPeripheral" {

                                let cell:AudioConfigTableViewCell = self.tableView.cellForRowAtIndexPath(cellPath) as!
                                AudioConfigTableViewCell

                                if row == self.peripheralsTableDataCopy[i].trueIndex {

                                    cell.speakerNameTextField.enabled = false

                                    if ( self.peripheralsTableDataCopy[i].peripheraSectionIndex == 0 ) {

                                        peripheral.senderName =
                                            cell.speakerNameTextField.text!

                                        senderName = peripheral.senderName
                                    }

                                    else {

                                        peripheral.speakerName =
                                            cell.speakerNameTextField.text!

                                        peripheral.senderName = senderName
                                    }

                                    break;
                                }

                            }
                        }
                    }
                }
            }
        }
    }

    @IBAction func submitConfigToolBarButtonClick(sender: AnyObject) -> Void {

        // find a better way of doing this?

        peripheralsTableDataCopy = data!.valueofCells()

        var sectionMasterSpeaker = 0
        var sectionRemoteSpeaker = 0

        for (var i = 0; i < peripheralsTableDataCopy.count; i++) {

            if !peripheralsTableDataCopy[i].isSection {
                if peripheralsTableDataCopy[i].peripheraSectionIndex == 0 {

                    sectionMasterSpeaker++

                }
                else if peripheralsTableDataCopy[i].peripheraSectionIndex == 1 {

                    sectionRemoteSpeaker++

                }
            }
        }

        if sectionMasterSpeaker != 1  {

            let alertView = UIAlertController(title: "Rule Check", message: "Master speaker section should have ONE peripheral!", preferredStyle: .Alert)
            alertView.addAction(UIAlertAction(title: "Ok", style: .Default, handler: nil))
            presentViewController(alertView, animated: true, completion: nil)

            return

        }

        else if sectionRemoteSpeaker < 1 {

            let alertView = UIAlertController(title: "Rule Check", message: "Remote speaker section must atleast contain ONE peripheral!", preferredStyle: .Alert)
            alertView.addAction(UIAlertAction(title: "Ok", style: .Default, handler: nil))
            presentViewController(alertView, animated: true, completion: nil)

            return

        }

        //also remove all the peripherals of the section 2 which is discovered peripherals
        //this includes the section also

        for (var j = peripheralsTableDataCopy.count-1; j >  0; j-- ) {

            if peripheralsTableDataCopy[j].peripheraSectionIndex == 2 {

                peripheralsTableDataCopy.removeAtIndex(j)

            }

        }

        // start doing the connect for each peripheral in the allPeripheralData list
        // this is the final list obtained after reordering

        var peripheral : Peripheral;

        for (var i = 0; i < peripheralsTableDataCopy.count; i++) {

            if !peripheralsTableDataCopy[i].isSection {
                peripheral = peripheralsTableDataCopy[i].peripheralMeta!
                CentralManager.sharedInstance().connectPeripheral(peripheral)
            }

        }

    }

    // MARK: UITextFieldDelegate

    func textFieldShouldReturn(textField: UITextField) -> Bool {
        // Hide the keyboard.
        textField.resignFirstResponder()
        return true
    }

    func textFieldDidEndEditing(textField: UITextField) {
        // Hide the keyboard.
        textField.resignFirstResponder()
    }

    func textFieldDidBeginEditing(textField: UITextField) {

    }

    func initBluetooth() {
        var myDict: NSDictionary?
        if let path = NSBundle.mainBundle().pathForResource("Keys", ofType: "plist") {
            myDict = NSDictionary(contentsOfFile: path)
        }
        if let dict = myDict {
            self.serviceUUIDString = dict["ServiceUUIDString"] as! String
            self.characteristicUUIDString = dict["NameCharacteristicUUIDString"] as! String
        }

        CentralManager.sharedInstance().connectPeripheralDelegate = self
    }

    // MARK: - Table view data source

    func tableView(tableView: UITableView, numberOfRowsInSection section: Int) -> Int {

        return data!.getRows()
    }

    func tableView(tableView: UITableView, cellForRowAtIndexPath indexPath: NSIndexPath) -> UITableViewCell {

        // all these computations are to give a text name to the peripheralsTableDataCopy
        // based on the ID
        let cellText = data!.cellText(indexPath)
        let cellID = data!.cellID(indexPath)

        if cellID == "cellPeripheralSection" {

            let cell = tableView.dequeueReusableCellWithIdentifier(cellID) as! AudioConfigTableViewSectionCell

            cell.textLabel?.text = cellText
            cell.backgroundColor = UIColor.clearColor()

            return cell
        }
        else if cellID == "cellPeripheral" {

            let cell = tableView.dequeueReusableCellWithIdentifier(cellID) as! AudioConfigTableViewCell

            //cell.textLabel?.text = cellText
            cell.speakerNameTextField.text = cellText
            cell.speakerNameTextField.enabled = false
            cell.statusLabel.text = ""
            cell.backgroundColor = UIColor.lightGrayColor()

            return cell
        }

        return UITableViewCell()
    }

    func numberOfSectionsInTableView(tableView: UITableView) -> Int {
        return 1
    }

    // MARK: ConnectPeripheralProtocol
    func didConnectPeripheral(cbPeripheral: CBPeripheral!) {
        Logger.debug("AppDelegate#didConnectPeripheral \(cbPeripheral.name)")
        if let peripheral = self.discoveredPeripherals[cbPeripheral] {

            //peripheral.discoverServices(<#T##serviceUUIDs: [CBUUID]!##[CBUUID]!#>, delegate: <#T##ReadPeripheralProtocol!#>)
            // look for only network service
            peripheral.discoverServices([CBUUID(string: serviceUUIDString)], delegate: self)

            }
    }

    func didDisconnectPeripheral(cbPeripheral: CBPeripheral!, error: NSError!, userClickedCancel: Bool) {
        Logger.debug("AppDelegate#didDisconnectPeripheral \(cbPeripheral.name)")
    }

    func didRestorePeripheral(peripheral: Peripheral) {
        Logger.debug("AppDelegate#didRestorePeripheral \(peripheral.name)")
    }

    func bluetoothBecomeAvailable() {
        self.startScanning()
        bluetoothOn = true
    }

    func bluetoothBecomeUnavailable() {

        //let alertView = UIAlertController(title: "Alert", message: "Please turn ON bluetooth and then try Again!", preferredStyle: .Alert)
        //alertView.addAction(UIAlertAction(title: "Ok", style: .Default, handler: nil))
        //presentViewController(alertView, animated: true, completion: nil)

        print("\nPlease turn ON bluetooth!\n")

        bluetoothOn = false

        self.stopScanning()
    }

    // MARK: Public functions
    func startScanning() {
        for peripheral:Peripheral in self.discoveredPeripherals.values {
            peripheral.isNearby = false
        }
        CentralManager.sharedInstance().startScanning(afterPeripheralDiscovered, allowDuplicatesKey: true)
    }

    func stopScanning() {
        CentralManager.sharedInstance().stopScanning()
    }

    func afterPeripheralDiscovered(cbPeripheral:CBPeripheral, advertisementData:NSDictionary, RSSI:NSNumber) {
        Logger.debug("AppDelegate#afterPeripheralDiscovered: \(cbPeripheral)")
        var peripheral : Peripheral

        if let p = discoveredPeripherals[cbPeripheral] {
            peripheral = p
        } else {
            peripheral = Peripheral(cbPeripheral:cbPeripheral, advertisements:advertisementData as Dictionary<NSObject, AnyObject>, rssi:RSSI.integerValue)
            discoveredPeripherals[peripheral.cbPeripheral] = peripheral
        }

        peripheral.isNearby = true

        NSNotificationCenter.defaultCenter().postNotificationName("afterPeripheralDiscovered", object: nil)
    }

    func getRequiredPeripheral(cbperipheral : CBPeripheral!) -> Peripheral? {

        for var peripheralIndex = 0; peripheralIndex < peripheralsTableDataCopy.count; peripheralIndex++ {

            if let test = peripheralsTableDataCopy[peripheralIndex].peripheralMeta {

                if test.identifier.isEqual(cbperipheral.identifier) {

                    // if no sender/source name provided by user then choosing a default one
                    if peripheralsTableDataCopy[peripheralIndex].peripheraSectionIndex == 0 {

                        if peripheralsTableDataCopy[peripheralIndex].peripheralMeta!.senderName.isEmpty {

                            peripheralsTableDataCopy[peripheralIndex].peripheralMeta!.senderName = "APOLLO"
                        }

                    }
                    // if no speaker name provided by user then choosing a default one
                    else {

                        if peripheralsTableDataCopy[peripheralIndex].peripheralMeta!.speakerName.isEmpty {

                            peripheralsTableDataCopy[peripheralIndex].peripheralMeta!.speakerName = "APOLLO"

                            if peripheralsTableDataCopy[peripheralIndex].peripheralMeta!.senderName.isEmpty {

                                peripheralsTableDataCopy[peripheralIndex].peripheralMeta!.senderName = "APOLLO"
                            }
                        }
                    }

                    return peripheralsTableDataCopy[peripheralIndex].peripheralMeta!;
                }
            }
        }
        return nil;
    }

    // MARK: ReadPeripheralProtocol

    func didDiscoverCharacteristicsofPeripheral(cbservice : CBService!) {

        if let peripheral = self.getRequiredPeripheral(cbservice.peripheral) {

            senderName = peripheral.senderName
            speakerName = peripheral.speakerName

            // its a source/sender
            if(!senderName.isEmpty && speakerName.isEmpty) {

                for charateristic in cbservice.characteristics! {

                    let thisCharacteristic = charateristic as CBCharacteristic

                    switch thisCharacteristic.UUID {

                    case ApolloConfigNWModeUUID:

                        print("ApolloConfigNWModeUUID")

                        var enableValue = 0
                        let enablyBytes = NSData(bytes: &enableValue, length: sizeof(UInt8))

                        // read of the characteristic helps to ensure write is successful
                        thisCharacteristic.service.peripheral.readValueForCharacteristic(thisCharacteristic)

                        print(thisCharacteristic.value)

                        // writing the mode value twice to ensure its written all the time
                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)

                        usleep(1000*1000)

                        // writing the mode value twice to ensure its written all the time
                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)

                    case ApolloConfigNWSSIDUUID:

                        print("ApolloConfigNWSSIDUUID")


                        // this celltext needs to be converted to NSData and then fed
                        let data = self.senderName.dataUsingEncoding(NSUTF8StringEncoding)

                        print("sender name:\(senderName)")

                        if let d = data {
                            thisCharacteristic.service.peripheral.writeValue(d, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)
                        }

                    case ApolloConfigNWPassphraseUUID:

                        print("ApolloConfigNWPassphraseUUID")

                        let str = "12345678"

                        // this celltext needs to be converted to NSData and then fed
                        let data = str.dataUsingEncoding(NSUTF8StringEncoding)

                        if let d = data {
                            thisCharacteristic.service.peripheral.writeValue(d, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)
                        }

                    case ApolloConfigSourceInputUUID:

                        print("ApolloConfigSourceInputUUID")

                        var enableValue = 1
                        let enablyBytes = NSData(bytes: &enableValue, length: sizeof(UInt8))

                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)

                    case ApolloConfigSourceInputVolumeUUID:

                        print("ApolloConfigSourceInputVolumeUUID")

                        var enableValue = 50
                        let enablyBytes = NSData(bytes: &enableValue, length: sizeof(UInt8))

                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)

                    default:
                        _ = 0

                    }
                }
                CentralManager.sharedInstance().cancelPeripheralConnection(peripheral, userClickedCancel: true);
                peripheral.senderName = ""
            }
            // its a speaker
            else if(!speakerName.isEmpty && !senderName.isEmpty) {

                for charateristic in cbservice.characteristics! {

                    let thisCharacteristic = charateristic as CBCharacteristic

                    switch thisCharacteristic.UUID {

                    case ApolloConfigNWModeUUID:

                        print("ApolloConfigNWModeUUID")

                        var enableValue = 1
                        let enablyBytes = NSData(bytes: &enableValue, length: sizeof(UInt8))

                        // read of the characteristic helps to ensure write is successful
                        thisCharacteristic.service.peripheral.readValueForCharacteristic(thisCharacteristic)

                        print(thisCharacteristic.value)

                        // writing the mode value twice to ensure its written all the time
                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)

                        usleep(1000*1000)

                        // writing the mode value twice to ensure its written all the time
                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)


                    case ApolloConfigNWSSIDUUID:

                        print("ApolloConfigNWSSIDUUID")

                        // this celltext needs to be converted to NSData and then fed
                        let data = self.senderName.dataUsingEncoding(NSUTF8StringEncoding)

                        if let d = data {
                            thisCharacteristic.service.peripheral.writeValue(d, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)
                        }

                    case ApolloConfigNWPassphraseUUID:

                        print("ApolloConfigNWPassphraseUUID")

                        let str = "12345678"

                        // this celltext needs to be converted to NSData and then fed
                        let data = str.dataUsingEncoding(NSUTF8StringEncoding)

                        if let d = data {
                            thisCharacteristic.service.peripheral.writeValue(d, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)
                        }

                    case ApolloConfigSpeakerNameUUID:

                        print("ApolloConfigSpeakerNameUUID")

                        let data = self.speakerName.dataUsingEncoding(NSUTF8StringEncoding)

                        print("speaker name:\(speakerName)")

                        if let d = data {
                            thisCharacteristic.service.peripheral.writeValue(d, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)
                        }

                    case ApolloConfigSpeakerChannelUUID:

                        print("ApolloConfigSpeakerChannelUUID")

                        var enableValue = 3
                        let enablyBytes = NSData(bytes: &enableValue, length: sizeof(UInt8))

                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)

                    case ApolloConfigSpeakerOutputVolumeUUID:

                        print("ApolloConfigSpeakerOutputVolumeUUID")

                        var enableValue = 50
                        let enablyBytes = NSData(bytes: &enableValue, length: sizeof(UInt8))

                        thisCharacteristic.service.peripheral.writeValue(enablyBytes, forCharacteristic: thisCharacteristic, type: CBCharacteristicWriteType.WithResponse)


                    default:
                        _ = 0
                    }
                }
                CentralManager.sharedInstance().cancelPeripheralConnection(peripheral, userClickedCancel: true);
                peripheral.speakerName = ""
            }
        }

    }

    func didWriteValueForCharacteristic(cbPeripheral: CBPeripheral!, characteristic: CBCharacteristic!, error: NSError!) {

        if let peripheral = self.discoveredPeripherals[cbPeripheral] {

            //CentralManager.sharedInstance().cancelPeripheralConnection(peripheral, userClickedCancel: true);
        }

        if let data = characteristic.value {
            if let result = NSString(data: data, encoding: NSUTF8StringEncoding) {
                Logger.debug("AppDelegate#didWriteValueForCharacteristic \(result)")
                let results = result.componentsSeparatedByString(" ")


                if let peripheral:Peripheral = discoveredPeripherals[cbPeripheral] {

                    peripheral.hasBeenConnected = true
                }

                NSNotificationCenter.defaultCenter().postNotificationName("didWriteValueForCharacteristic", object: nil)
            }
        } else {
            Logger.debug("AppDelegate#didWriteValueForCharacteristic: Received nil characteristic value from peripheral \(cbPeripheral.name)")
        }
        if let peripheral = self.discoveredPeripherals[cbPeripheral] {
            Logger.debug("AppDelegate#didWriteValueForCharacteristic: Cancel peripheral connection")

        }

        print("Network characterstic value written")

    }

    // is this used at all?
    func didUpdateValueForCharacteristic(cbPeripheral: CBPeripheral!, characteristic: CBCharacteristic!, error: NSError!) {

        if let data = characteristic.value {
            if let result = NSString(data: data, encoding: NSUTF8StringEncoding) {
                Logger.debug("AppDelegate#didUpdateValueForCharacteristic \(result)")
                let results = result.componentsSeparatedByString(" ")
                //let username:String = results[0] as String
                //let installationId = results[1] as String

                if let peripheral:Peripheral = discoveredPeripherals[cbPeripheral] {
                    //peripheral.name = username
                    //peripheral.installationId = installationId
                    peripheral.hasBeenConnected = true
                }

                NSNotificationCenter.defaultCenter().postNotificationName("didUpdateValueForCharacteristic", object: nil)
            }
        } else {
            Logger.debug("AppDelegate#didUpdateValueForCharacteristic: Received nil characteristic value from peripheral \(cbPeripheral.name)")
        }
        if let peripheral = self.discoveredPeripherals[cbPeripheral] {
            Logger.debug("AppDelegate#didUpdateValueForCharacteristic: Cancel peripheral connection")
            CentralManager.sharedInstance().cancelPeripheralConnection(peripheral, userClickedCancel: true);
        }
    }

    @IBAction func longPressGestureRecognized(sender: AnyObject) {

        let longPress: UILongPressGestureRecognizer = sender as! UILongPressGestureRecognizer
        let state: UIGestureRecognizerState = longPress.state
        var location: CGPoint = longPress.locationInView(self.tableView)

        if (self.tableView.indexPathForRowAtPoint(location) == nil) {
            print("tableview = nil")
            location = prevLocation!
        }
        let indexPath: NSIndexPath = self.tableView.indexPathForRowAtPoint(location)!
        prevLocation = location

        switch (state) {

        case .Began:

            sourceIndexPath = indexPath

            let cell: UITableViewCell = self.tableView.cellForRowAtIndexPath(indexPath)!
            snapshot = self.customSnapshoFromView(cell)

            var center: CGPoint
            center = cell.center
            snapshot!.center = center
            snapshot!.alpha = 0.0
            self.tableView.addSubview(snapshot!)
            UIView.animateWithDuration(0.25, animations: {
                center.y = location.y
                self.snapshot!.center = center
                self.snapshot!.transform = CGAffineTransformMakeScale(1.05, 1.05)
                self.snapshot!.alpha = 0.98
                cell.alpha = 0.0
                }, completion: {(finished: Bool) in cell.hidden = true
            })

            break;

        case .Changed:
            var center: CGPoint = snapshot!.center
            center.y = location.y
            snapshot!.center = center
            if !indexPath.isEqual(sourceIndexPath) {

                data!.completeMove(sourceIndexPath!, destination: indexPath)

                self.tableView.moveRowAtIndexPath(sourceIndexPath!, toIndexPath: indexPath)

                print("Moving row \(sourceIndexPath!.row) to row \(indexPath.row)")

                sourceIndexPath = indexPath
            }

            break;

        default:

            let cell: UITableViewCell = self.tableView.cellForRowAtIndexPath(sourceIndexPath!)!
            cell.hidden = false
            cell.alpha = 0.0
            UIView.animateWithDuration(0.25, animations: {
                self.snapshot!.center = cell.center
                self.snapshot!.transform = CGAffineTransformIdentity
                self.snapshot!.alpha = 0.0
                cell.alpha = 1.0

                }, completion: {(finished: Bool) in self.sourceIndexPath = nil
                    self.snapshot!.removeFromSuperview()
                    self.snapshot = nil

            })

            break;

        }

    }
    func customSnapshoFromView(inputView: UIView) -> UIView {
        UIGraphicsBeginImageContextWithOptions(inputView.bounds.size, false, 0)
        inputView.layer.renderInContext(UIGraphicsGetCurrentContext()!)
        let image: UIImage = UIGraphicsGetImageFromCurrentImageContext()
        UIGraphicsEndImageContext()
        let snapshot: UIView = UIImageView(image: image)
        snapshot.layer.masksToBounds = false
        snapshot.layer.cornerRadius = 0.0
        snapshot.layer.shadowOffset = CGSizeMake(-5.0, 0.0)
        snapshot.layer.shadowRadius = 5.0
        snapshot.layer.shadowOpacity = 0.4
        return snapshot
    }
}
