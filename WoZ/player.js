// ============================================================================
// player.js
// ============================================================================
//
//  Created by leuski on 5/5/15.
//  Copyright (c) 2015 Anton Leuski and ICT. All rights reserved.
// ============================================================================

var vhmsgView;

$(document).ready(function () {

	vhmsgView = new VHMSGView(new VHMSG());

	$(window).on("unload", function () {
		if (vhmsgView)
			vhmsgView.vhmsg.disconnect();
	});


	vhmsgView.vhmsg.subscribe('vrExpress', function (messageBody) {
		var n = messageBody.indexOf("<?");
		if (n < 0) return;
		var xmlText = messageBody.substring(n);
		var xmlDoc = $.parseXML(xmlText);
		var $speech = $(xmlDoc).find("speech");
		if ($speech) {
			$('#player').text($speech.text());
		} else {
			$('#player').text("");
		}
	});


	vhmsgView.connect();
});

