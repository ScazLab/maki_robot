// ============================================================================
// TemplateEditor.js
// ============================================================================
//
//  Created by leuski on 5/6/15.
//  Copyright (c) 2015 Anton Leuski and ICT. All rights reserved.
// ============================================================================

function TemplateEditor() {
	this.send = function() {};
}

TemplateEditor.prototype.stringNeedsExpansion = function (text) {
	return text.match(/##input##/);
};

TemplateEditor.prototype.display = function (text) {
	text = text.replace(/##input##/g, "<input type='text'>");

	var messageElement = $('#TemplateEditor_message');
	messageElement.html(text);

	var _this = this;

	var handleKeys = function (e) {
		var keyCode = ((typeof e.keyCode !='undefined' && e.keyCode) ? e.keyCode : e.which);
		if (keyCode === 27) {
			$("#TemplateEditor_cancel").click();
//			_this.doCancel();
		} else if (keyCode === 13) {
			$("#TemplateEditor_send").click();
//			_this.doSend();
		}
	};

	messageElement
			.find("input")
			.each(function(index, element) {
				//noinspection SpellCheckingInspection
				$(element).on("keydown", handleKeys);
			});

	var editor = $('#TemplateEditor');

	editor.css("margin-top", -editor.height()/2);
	editor.css("margin-left", -editor.width()/2);
	editor.show("fast");
};

TemplateEditor.prototype.doCancel = function () {
	$('#TemplateEditor').hide("fast");
};

TemplateEditor.prototype.doSend = function () {
	var text = $('#TemplateEditor_message')
			.contents()
			.map(function(index, element) {
				return $(element).is("input") ? $(element).val() : $(element).text();
			})
			.get()
			.join("");
	this.send(text);
	$('#TemplateEditor').hide("fast");
};
