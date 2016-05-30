// ============================================================================
// woz.js
// ============================================================================
//
//  Created by leuski on 5/6/15.
//  Copyright (c) 2015 Anton Leuski and ICT. All rights reserved.
// ============================================================================

function DelayedSearch(inResultDivID, inLabel, inParent, inSearchCallback) {
	this.timer = null;
	this.query = null;
	this.callback = inSearchCallback;
	this.divID = inResultDivID;
	this.label = inLabel;
	this.parent = inParent;
}

DelayedSearch.prototype.search = function (inText, inDelay) {
	this.query = $.trim(inText);
	if (!this.timer) {
		var _this = this;
		this.timer = window.setTimeout(function () {
			_this.timer = null;
			_this.callback(_this.query);
		}, inDelay);
	}
};

DelayedSearch.prototype.presentResults = function (inResultButtonArray) {
	var theSearchResultsElement = $(this.divID);
	theSearchResultsElement.empty();

	if (inResultButtonArray.length > woz.maxResultCount) {
		inResultButtonArray = inResultButtonArray.slice(0, this.parent.maxResultCount);
	}

	while (inResultButtonArray.length < this.parent.maxResultCount) {
		inResultButtonArray.push("__placeholder__");
	}

	if (inResultButtonArray.length > 0) {
		this.parent.wozView.append_html_of_row_with_label_and_button_ids_on_screen_with_id_to_html_element(this.label,
				inResultButtonArray, "", theSearchResultsElement);
	}
};

function WoZ() {

	this.setupNetwork();
	this.messageCount = 0;
	this.editor = new TemplateEditor();

	this.maxResultCount = 8;

	var _this = this;

	var dataURL = "../data/master-table.xls";

	if (defined(globalDataFileURL)) {
		dataURL = globalDataFileURL;
	}

	this.loadData(dataURL, function (inData) {

		var screen_ids = [];
		for (var screen_id in inData.screens) {
			if (inData.screens.hasOwnProperty(screen_id)) {
				screen_ids.push(screen_id);
			}
		}
		inData.allScreenIDs = screen_ids;
		var firstScreenID = screen_ids.length > 0 ? screen_ids[0] : undefined;

		_this.model = inData;
		_this.wozView = new WoZView(_this.model);

		_this.renderAllScreens();

// set this to true to show all screens (and print)
		var I_want_to_show_all_screens = false;
		if (I_want_to_show_all_screens) {
			_this.presentAllScreens();
		} else {
			_this.presentScreen(firstScreenID);
		}

	});


	this.regexSearch = new DelayedSearch("#search_results", "Search Results", this, function (text) {
		_this.regexSearch.presentResults(_this.buttonsMatchingRegexQuery(text, _this.maxResultCount));
	});

	this.jeromeSearch = new DelayedSearch("#jerome_results", "Jerome Results", this, function (text) {
		_this.queryNPCEditorWithText(text, "type");
	});

	this.speechSearch = new DelayedSearch("#speech_results", "Speech Results", this, function (text) {
		_this.queryNPCEditorWithText(text, "speech");
	});

	var html_searchButton = $('#search_button');

	//noinspection SpellCheckingInspection
	html_searchButton.on('keydown', function () {
		_this.searchFor($(this).val(), 500);
	});

	html_searchButton.on('change paste input', function () {
		_this.searchFor($(this).val(), 0);
	});
}

WoZ.prototype._loadDataFromExcelURL = function (inURL, inHandler) {

	var oReq = new XMLHttpRequest();
	oReq.open("GET", inURL, true);
	//noinspection SpellCheckingInspection
	oReq.responseType = "arraybuffer";

	//noinspection SpellCheckingInspection
	oReq.onload = function () {
		var arrayBuffer = oReq.response;

		/* convert data to binary string */
		var data = new Uint8Array(arrayBuffer);
		var arr = [];
		for (var i = 0; i != data.length; ++i) arr[i] = String.fromCharCode(data[i]);
		var binaryStr = arr.join("");

		/* Call XLSX */
		var workbook = XLSX.read(binaryStr, {type: "binary"});
		var reader = new ExcelReader(true);
		var modelData = reader.convert_workbook_to_json(workbook);
		inHandler(modelData);
	};

	oReq.send();
};

/**
 * reads WoZ data from the given URL. Handler should accept a data object
 * @param inURL
 * @param inHandler
 */
WoZ.prototype.loadData = function (inURL, inHandler) {
	if (inURL.toLowerCase().endsWith(".json")) {
		$.getJSON(inURL, function (data) {
			inHandler(data);
		});
	} else {
		this._loadDataFromExcelURL(inURL, inHandler);
	}
};

WoZ.prototype.setupNetwork = function () {

	this.vhmsgui = new VHMSGView();

	var _this = this;

	this.vhmsgui.vhmsg.subscribe('BatchUtteranceMatches', function (inMessageBody) {
//			console.log("message: " + messageBody);
		var xmlDoc = $.parseXML(inMessageBody);
		var resultsIDs = $(xmlDoc).find('responses').map(function () {
			return $(this).attr('ID');
		}).get();

		var result = $(xmlDoc).find('utterance>field[name="ID"]').map(function () {
			return $(this).text();
		}).get();

		if (resultsIDs.length && resultsIDs[0] == "speech") {
			_this.speechSearch.presentResults(result);
		} else {
			_this.jeromeSearch.presentResults(result);
		}

	});

	this.vhmsgui.vhmsg.subscribe('vrSpeech', function (inMessageBody) {
		var args = inMessageBody.split(" ");
		if (args.length <= 5) return;
		//noinspection SpellCheckingInspection
		if (args[0] == "interp" || args[0] == "partial") {
			var message = args.slice(5).join(" ");
			_this.speechSearch.search(message, 500);
		}
	});

	this.vhmsgui.connect();
};

WoZ.prototype.queryNPCEditorWithText = function (inText, inSource) {
	this.vhmsgui.vhmsg.send('BatchUtterances', '<requests ID="'
			+ inSource
			+ '" agentName="captain"><request source="woz"><field name="text">'
			+ inText.encodeHTML()
			+ '</field></request></requests>');
};


WoZ.prototype.button_matches_query = function (inButtonModel, inRegex) {
	for (var badge_id in inButtonModel.badges) {
		if (inButtonModel.badges.hasOwnProperty(badge_id)) {
			if (inButtonModel.badges[badge_id].search(inRegex) >= 0) {
				return true
			}
		}
	}

	return (defined(inButtonModel.tooltip) && inButtonModel.tooltip.search(inRegex) >= 0)
			|| (defined(inButtonModel.label) && inButtonModel.label.search(inRegex) >= 0)
};

WoZ.prototype.buttonsMatchingRegexQuery = function (inQuery, inMaxResultCount) {

	var result = [];
	inQuery = $.trim(inQuery);
	if (inQuery.length == 0) return result;

	var regex = new RegExp(inQuery, "gi");

	for (var button_id in this.model.buttons) {
		if (this.model.buttons.hasOwnProperty(button_id)) {
			var theButton = this.model.buttons[button_id];
			if (this.button_matches_query(theButton, regex)) {
				result.push(button_id);
				if (result.length >= inMaxResultCount) {
					break;
				}
			}
		}
	}

	return result;
};

WoZ.prototype.searchFor = function (inQuery, inDelay) {
	this.regexSearch.search(inQuery, inDelay);
	this.jeromeSearch.search(inQuery, inDelay);
};

WoZ.prototype.windowWillClose = function () {
	if (this.vhmsgui)
		this.vhmsgui.vhmsg.disconnect();
};

// present all screens
WoZ.prototype.renderAllScreens = function () {
	var html_screenAreaDiv = $("#screen");
	html_screenAreaDiv.empty();
	this.wozView.append_html_of_all_screens_to_html_element(html_screenAreaDiv);
};

// present screen with ID
WoZ.prototype.presentScreen = function (screen_id) {
	this.wozView.hide_screen_with_id(this.currentScreenID);
	this.currentScreenID = screen_id;
	this.wozView.show_screen_with_id(this.currentScreenID);
};

// present all screens
WoZ.prototype.presentAllScreens = function () {
	this.wozView.show_all_screens();
};

//noinspection JSUnusedGlobalSymbols
WoZ.prototype.handleButtonClick = function (inButtonID, inScreenID) {
	var buttonModel = this.model.buttons[inButtonID];

	if (!defined(buttonModel)) {
		return false;
	}

	var targetID = undefined;
	if (defined(buttonModel.transitions)) {
		targetID = buttonModel.transitions[inScreenID];
		if (!defined(targetID)) {
			targetID = buttonModel.transitions["_any"];
		}
		if (defined(targetID)) {
			this.presentScreen(targetID);
			return false;
		}
	}

	if (!defined(this.vhmsgui.vhmsg)) return;

	var buttonModelCopy = {};
	for (var attr in buttonModel) {
		if (buttonModel.hasOwnProperty(attr))
			buttonModelCopy[attr] = buttonModel[attr];
	}

	if (!defined(buttonModelCopy.tooltip))
		buttonModelCopy.tooltip = "";

	buttonModelCopy.id = inButtonID;

	var h = "000000" + Number(this.messageCount++).toString();
	buttonModelCopy.messageCount = h.substr(h.length - 5, 5);

	if (!defined(buttonModelCopy.vhmsg)) {
		if (!defined(buttonModelCopy.sender)) {
			buttonModelCopy.sender = "visitor";
		}
		if (!defined(buttonModelCopy.addressee)) {
			buttonModelCopy.addressee = "all";
		}
		//noinspection SpellCheckingInspection
		buttonModelCopy.vhmsg = 'vrExpress $$sender$$ $$addressee$$ $$sender$$$$messageCount$$ '
				+ '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'
				+ '<act><participant id="$$XML(sender)$$" role="actor" /><fml><turn start="take" end="give" />'
				+ '<affect type="neutral" target="addressee"></affect><culture type="neutral"></culture>'
				+ '<personality type="neutral"></personality></fml><bml><speech id="sp1" ref="'
				+ '$$XML(id)$$'
				+ '" type="application/ssml+xml">'
				+ '$$XML(tooltip)$$'
				+ '</speech></bml></act>';
	}

	if (this.editor.stringNeedsExpansion(buttonModelCopy.tooltip)) {
		var _this = this;
		this.editor.send = function (text) {
			buttonModelCopy.tooltip = text;
			_this.send(buttonModelCopy);
		};
		this.editor.display(buttonModelCopy.tooltip);
	} else {
		this.send(buttonModelCopy);
	}

	return false;
};

WoZ.prototype.send = function (inButtonModel) {

	var message = inButtonModel.vhmsg.replace(/\$\$XML\((.*?)\)\$\$/g, function (match, p1) {
		var text = inButtonModel[p1];
		if (!defined(text)) text = "";
		return text.encodeHTML();
	});

	message = message.replace(/\$\$(.*?)\$\$/g, function (match, p1) {
		var text = inButtonModel[p1];
		if (!defined(text)) text = "";
		return text;
	});

	this.vhmsgui.vhmsg.send(message);

};

var woz;

$(document).ready(function () {

	var supported = ("WebSocket" in window);
	if (!supported) {
		var msg = "Your browser does not support Web Sockets. This app will not work properly.<br>";
		msg += "Please use a Web Browser with Web Sockets support (WebKit or Google Chrome).";
		$("#connect").html(msg);
	}

	woz = new WoZ();

	var setup_body_size = function () {
		var body = document.getElementsByTagName('body');
		$(body).css('width', window.innerWidth + 'px');
	};

	window.onresize = setup_body_size;
	setup_body_size();

	$(window).on("unload", function () {
		woz.windowWillClose();
	});

});