//
//  AudioConfigTableViewCell.swift
//  apollo_config

/*
 * $ Copyright Broadcom Corporation $
 */

import UIKit

class AudioConfigTableViewCell: UITableViewCell {
    
    @IBOutlet weak var audioImageView: UIImageView!
    @IBOutlet weak var speakerNameTextField: UITextField!
    @IBOutlet weak var statusLabel: UILabel!

    override func awakeFromNib() {
        super.awakeFromNib()
        // Initialization code
    }

    override func setSelected(selected: Bool, animated: Bool) {
        super.setSelected(selected, animated: animated)

        // Configure the view for the selected state
    }

}
