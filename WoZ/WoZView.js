// ============================================================================
// WoZView.js
// ============================================================================
//
//  Created by leuski on 5/6/15.
//  Copyright (c) 2015 Anton Leuski and ICT. All rights reserved.
// ============================================================================

function WoZView(modelData) {
	this.model = modelData;
	this.buttonLabelMinimumFontSize = 5;
}

WoZView.prototype.append_html_of_button_with_id_on_screen_with_id_to_html_element = function (button_id, screen_id, html_element) {
	var buttonModel = this.model.buttons[button_id];

	if (!defined(buttonModel)) {

		if (button_id == "__placeholder__") {
			var buttonDiv = $('<div class="button placeholder">');
			html_element.append(buttonDiv);
			return buttonDiv;
		}

		return undefined;
	}

	var theToolTip = defined(buttonModel.tooltip) ? buttonModel.tooltip : '';
	var theLabel = defined(buttonModel.label) ? buttonModel.label : '';
	theLabel = $.trim(theLabel);

	var command = 'woz.handleButtonClick(\'' + button_id.encodeJS() + '\', \'' + screen_id.encodeJS() + '\')';

	var theColor = this.model.colors[buttonModel.color];
	var colorCss = "";
	var hex = function (x) {
		var h = "000000" + Number(x).toString(16);
		return h.substr(h.length - 2, 2);
	};

	if (defined(theColor)) {
		colorCss = ' style="background:' + '#' + hex(theColor.r) + hex(theColor.g) + hex(theColor.b) + ';"';
	}
	var theButtonDiv = $('<div class="button selectable" title="' + theToolTip.encodeHTML()
	+ '" onclick="' + command.encodeHTML() + ';"' + colorCss + '>');
	html_element.append(theButtonDiv);

	for (var badge_id in buttonModel.badges) {
		if (buttonModel.badges.hasOwnProperty(badge_id)) {
			theButtonDiv.append($('<span class="badge ' + badge_id + '">' + buttonModel.badges[badge_id].encodeHTML() + '</span>'));
		}
	}

	var span = $('<span class="button-label">' + theLabel.encodeHTML() + '</span>');
	theButtonDiv.append(span);

	if (!defined(buttonModel.fontSize)) {
// search for the right font size
		var smallestUnacceptableFontSize = parseInt(span.css('font-size'));
		var largestAcceptableFontSize = this.buttonLabelMinimumFontSize;
		var fontSize = smallestUnacceptableFontSize;
		while (fontSize > largestAcceptableFontSize) {
			span.css('font-size', fontSize.toString() + 'px');
			if (span.width() <= theButtonDiv.width() && span.height() <= theButtonDiv.height()) {
				if (fontSize >= (smallestUnacceptableFontSize - 1)) {
					break;
				} else {
					largestAcceptableFontSize = fontSize;
				}
			} else {
				smallestUnacceptableFontSize = fontSize;
			}
			fontSize = Math.floor((largestAcceptableFontSize + smallestUnacceptableFontSize) / 2);
		}
		buttonModel.fontSize = fontSize;
	}
	span.css('font-size', buttonModel.fontSize.toString() + 'px');

	return theButtonDiv;

};

WoZView.prototype.append_html_of_row_with_label_and_button_ids_on_screen_with_id_to_html_element = function (row_label, button_ids, screen_id, html_element) {

	var theRowHeader = $('<div class="row-header">');
	html_element.append(theRowHeader);

	theRowHeader.append(row_label);

	var theRowContent = $('<div class="row-content">');
	html_element.append(theRowContent);

	var buttonArrayLength = button_ids.length;
	for (var j = 0; j < buttonArrayLength; ++j) {
		this.append_html_of_button_with_id_on_screen_with_id_to_html_element(button_ids[j], screen_id, theRowContent);
	}
};

WoZView.prototype.append_html_of_screen_with_id_to_html_element = function (screen_id, html_element) {

	window.document.title = screen_id;

	var screenModel = this.model.screens[screen_id];

	var theScreenHTML = $('<div class="screen">');
	html_element.append(theScreenHTML);

	var screenTitle = defined(screenModel.label) ? screenModel.label : screen_id;
	theScreenHTML.append($('<div class="screen-title">' + screenTitle + '</div>'));

	var table = $('<div>');
	theScreenHTML.append(table);

	var rowArrayLength = screenModel.rows.length;
	for (var i = 0; i < rowArrayLength; ++i) {
		var rowModel = this.model.rows[screenModel.rows[i]];

//		console.log(screenModel.rows[i]);

		var theRowElement = $('<div class="' + ( (i % 2 == 1) ? 'odd' : 'even' ) + '">');
		table.append(theRowElement);

		this.append_html_of_row_with_label_and_button_ids_on_screen_with_id_to_html_element(rowModel.label, rowModel.buttons, screen_id, theRowElement);
	}

	return theScreenHTML;
};

WoZView.prototype.append_html_of_all_screens_to_html_element = function (html_element) {
	this.allScreens = {};
	var index = 0;
	for (var screen_id in this.model.screens) {
		if (this.model.screens.hasOwnProperty(screen_id)) {
			this.allScreens[screen_id] = this.append_html_of_screen_with_id_to_html_element(screen_id, html_element);
			this.allScreens[screen_id].css('display', 'none');
			html_element.append($('<p style="page-break-before: always"> </p>'));
			index++;
			if (index > 100) {
				break;
			}
		}
	}
};

WoZView.prototype.show_hide_screen_with_id = function (screen_id, show) {
	if (!defined(screen_id)) return;

	window.document.title = screen_id;
	var table = this.allScreens[screen_id];
	if (defined(table)) {
		table.css('display', show ? 'block' : 'none');
	}
};

WoZView.prototype.show_hide_screens_with_ids = function (screen_ids, show) {
	if (!defined(screen_ids)) return;

	var n = screen_ids.length;
	for (var i = 0; i < n; ++i) {
		var table = this.allScreens[screen_ids[i]];
		if (defined(table)) {
			table.css('display', show ? 'block' : 'none');
		}
	}
};

WoZView.prototype.show_screen_with_id = function (screen_id) {
	this.show_hide_screen_with_id(screen_id, true);
};

WoZView.prototype.hide_screen_with_id = function (screen_id) {
	this.show_hide_screen_with_id(screen_id, false);
};

WoZView.prototype.show_all_screens = function () {
	this.show_hide_screens_with_ids(this.model.allScreenIDs, true);
};

//WoZView.prototype.hide_all_screens = function () {
//	this.show_hide_screen_with_id(this.allScreenIDs(), false);
//}
