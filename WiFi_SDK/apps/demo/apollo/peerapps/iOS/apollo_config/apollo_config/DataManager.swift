//
//  DataManager.swift
//  apollo_config

/*
 * $ Copyright Broadcom Corporation $
 */

import Foundation
import CoreBluetooth

let peripheralSectionCount = 3
let peripheralCountArray : [Int] = [ 0, 0, 3]
let sectionHeaderName : [String] = ["Master Speaker (Connect via BT)", "Remote Speaker", "Discovered Speakers"]

struct peripheralData {
    var name = ""
}

struct peripheralSectionData {
    var name = ""
}

struct cellData {
    // The absolute location of the cell in the tableview (which only has one section)
    var trueIndex = -1
    // Speaker: The location of the cell with respect to its section (maybe unused)
    // peripheralSection: The index of the peripheralSection in an array of all peripheralSections
    var peripheraSectionIndex = -1
    // Section or row
    var isSection = false
    // Speaker count (either 1 for a speaker or # of speakers in a trip)
    var peripheralCount = 0
    // Speaker metadata is populated for speaker cells or nil for peripheralSections
    var peripheralMeta: Peripheral?
    // peripheralSection metadata is populated for peripheralSection cells or nil for speakers
    var peripheralSectionMeta: peripheralSectionData?
    
    mutating func updateTrueIndex(index: Int) {
        trueIndex = index
    }
    
    mutating func updateperipheraSectionIndex(index: Int) {
        peripheraSectionIndex = index
    }
}

// This class provides editing touch recognition for the peripheralSection cell
class peripheralSectionCell: UITableViewCell {
    var indexPath = NSIndexPath()
    
    required init(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)!
    }
    
    override func layoutSubviews() {
        super.layoutSubviews()
        
    }
}

// This class maintains the tableView datasource
class DataManager {
    var cellHistory = [cellData]()
    var cells : Array<cellData> = []
    var cellsCollapsed = [cellData]()
    var movingperipheralSection = false
    
    var peripheralList : Dictionary<CBPeripheral, Peripheral> = [:]
    
    init(test : Dictionary<CBPeripheral, Peripheral>) {
        peripheralList = test
        addRandomData(peripheralSectionCount, peripheralsPer: peripheralCountArray)
    }
    
    func valueofCells() -> Array<cellData> {
        return self.cells
    }
    
    func addRandomData(peripheralSections: Int, peripheralsPer: [Int]) {
        
        var peripheralOffset = 0
        let count = peripheralList.count
        
        for peripheralSection in 0...2 {
            let peripheralSectionMeta = peripheralSectionData(name: "\(sectionHeaderName[peripheralSection])")
            let peripheralCountVal = peripheralsPer[peripheralSection]
            let cellMeta = cellData(trueIndex: peripheralSection + peripheralOffset, peripheraSectionIndex: peripheralSection, isSection: true, peripheralCount: peripheralCountVal, peripheralMeta: nil, peripheralSectionMeta: peripheralSectionMeta)
            
            cells.append(cellMeta)
            print("\(peripheralSectionMeta.name) @ row \(peripheralSection + peripheralOffset)")

            for peripheral in 0...count-1 {
                // rather than this static generation we have to resort to dynamic generation
                let key = Array(peripheralList.keys)[peripheral]
                
                print(peripheralList[key]!.name)
                
                let cellMeta = cellData(trueIndex: peripheralSection + peripheralOffset + 1, peripheraSectionIndex: peripheralSection, isSection: false, peripheralCount: 1, peripheralMeta: peripheralList[key]!, peripheralSectionMeta: nil)
                
                if peripheralSection != 0 && peripheralSection != 1 {
                    cells.append(cellMeta)
                    peripheralOffset++
                }
            }
            
        }
    }
    
    func getRows() -> Int {
        if movingperipheralSection {
            return cellsCollapsed.count
        }
        else {
            return cells.count
        }
    }
    
    func cellID(indexPath: NSIndexPath) -> String {
        var curCells = [cellData]()
        
        if movingperipheralSection {
            curCells = cellsCollapsed
        }
        else {
            curCells = cells
        }
        
        if curCells[indexPath.row].isSection {
            return "cellPeripheralSection"
        }
        
        return "cellPeripheral"
    }
    
    func cellText(indexPath: NSIndexPath) -> String {
        var curCells = [cellData]()
        
        if movingperipheralSection {
            curCells = cellsCollapsed
        }
        else {
            curCells = cells
        }
        
        if let peripheralSectionName = curCells[indexPath.row].peripheralSectionMeta?.name {
            return peripheralSectionName
        }
        else if let peripheral = curCells[indexPath.row].peripheralMeta {
            return peripheral.name
        }
        
        return "--ERROR--"
    }
    
    func completeMove(origin: NSIndexPath, destination: NSIndexPath) {
        // Determine direction of the move and grab the cell data
        let movedUp = origin.row > destination.row ? true : false
        let cellData = cells[origin.row]

        if !movingperipheralSection {
            let startRow = origin.row
            let endRow = destination.row
            
            if movedUp && startRow != endRow {
                var curSection = cellData.peripheraSectionIndex
                var passedFirst = false
                
                for (var i = startRow; i >= endRow; i--) {
                    // passed cells index getting incremented as you move up
                    cells[i].trueIndex++
                    
                    
                    // the cell you are moving up index keeps decrementing while you move up
                    cells[startRow].trueIndex--
                    
                    if cells[i].isSection && !passedFirst {
                        passedFirst = true
                        curSection--
                        cells[i].peripheralCount--
                    }
                }
                
                for (var i = endRow; i >= 0; i--) {
                    if cells[i].isSection {
                        cells[i].peripheralCount++
                        break
                    }
                }
                
                cells[startRow].peripheraSectionIndex = curSection
            }
            else {
                var curSection = cellData.peripheraSectionIndex
                _ = false
                
                for (var i = startRow; i <= endRow; i++) {
                    cells[i].trueIndex--
                    cells[startRow].trueIndex++
                    
                    if cells[i].isSection {
                        curSection++
                    }
                }
                
                for (var i = 0; i < cells.count; i++) {
                    if cells[i].peripheraSectionIndex == curSection && cells[i].isSection {
                        cells[i].peripheralCount++
                    }
                    
                    if cells[i].peripheraSectionIndex == cellData.peripheraSectionIndex && cells[i].isSection {
                        cells[i].peripheralCount++
                    }
                }
                
                cells[startRow].peripheraSectionIndex = curSection
            }
            
            rectifyTrueIndices()
        }
    }
    
    func rectifyTrueIndices() {
        var newCells = [cellData]()
        var curCell = 0
        var curTrueIndex = 0
        
        while (newCells.count != cells.count) {
            if cells[curCell].trueIndex == curTrueIndex {
                newCells.append(cells[curCell])
                curTrueIndex++
            }
            
            curCell++
            
            if curCell == cells.count {
                curCell = 0
            }
        }
        
        cells = newCells
    }
}