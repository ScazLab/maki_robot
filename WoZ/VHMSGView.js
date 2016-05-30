// ============================================================================
// VHMSGView.js
// ============================================================================
//
//  Created by leuski on 5/6/15.
//  Copyright (c) 2015 Anton Leuski and ICT. All rights reserved.
// ============================================================================


function VHMSGView() {
	this.initialize(new VHMSG());
}

//function VHMSGView(vhmsg) {
//	this.initialize(vhmsg);
//}

VHMSGView.prototype.initialize = function (vhmsg) {
	this.vhmsg = vhmsg;

	var html_connectURLInputField = $('#connect_url');
	var _this = this;

	// set the address to the host name
	html_connectURLInputField.val('localhost');
	if (location.host) {
		html_connectURLInputField.val(location.host);
	}

	this.vhmsg.onConnect = function () {
		$("#vhmsg_error").css("display", "none");
		_this.animateConnect();
	};

	this.vhmsg.onDisconnect = function (normal) {
		_this.animateDisconnect();
		if (!normal) {
			$("#vhmsg_error").css("display", "block");
		}
	};

	this.vhmsg.onError = function (error) {
		console.log("error: " + error);
	};

	this.vhmsg.debug = function (error) {
//			console.log("debug: " + error);
	};

	$('#connect_form').submit(function (event) {
		event.preventDefault();
		try {
			if (_this.vhmsg.isConnected()) {
				_this.vhmsg.disconnect();
			} else {
				_this.connect();
			}
		} catch (err) {
			// lets catch the errors here. Because if something throws,
			// and we do not catch it, the browser will "advance"
			// to the next page, resetting everything.
			console.error(err);
		}
		return false;
	});

	$('#disconnect_form').submit(function () {
		_this.vhmsg.disconnect();
		return false;
	});
};

VHMSGView.prototype.connect = function () {
	this.vhmsg.connect($("#connect_url").val(), $("#scope").val(), false);
};

VHMSGView.prototype.animateConnect = function () {
	this.animate_connect_form('#connect_form', '#disconnect_form');
};

VHMSGView.prototype.animateDisconnect = function () {
	this.animate_connect_form('#disconnect_form', '#connect_form');
};

VHMSGView.prototype.animate_connect_form = function (from_state, to_state) {
	$(from_state).hide();
	$(to_state).show();
};
