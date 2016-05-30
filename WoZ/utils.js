// ============================================================================
// TemplateEditor.js
// ============================================================================
//
//  Created by leuski on 3/22/16.
//  Copyright (c) 2016 Anton Leuski and ICT. All rights reserved.
// ============================================================================

if (typeof String.prototype.endsWith !== 'function') {
	String.prototype.endsWith = function(suffix) {
		return this.indexOf(suffix, this.length - suffix.length) !== -1;
	};
}

if (typeof String.prototype.startsWith !== 'function') {
	String.prototype.startsWith = function(prefix) {
		return this.indexOf(prefix) == 0;
	};
}

if (!String.prototype.encodeHTML) {
	String.prototype.encodeHTML = function () {
		return this.replace(/&/g, '&amp;')
				.replace(/</g, '&lt;')
				.replace(/>/g, '&gt;')
				.replace(/"/g, '&quot;')
				.replace(/'/g, '&#39;');
	};
}

if (!String.prototype.encodeJS) {
	String.prototype.encodeJS = function () {
		return this.replace(/\\/g, '\\\\')
				.replace(/'/g, '\\\'');
	};
}

if (!String.prototype.trim) {
	String.prototype.trim = function () {
		return this.replace(/^\s\s*/, '').replace(/\s\s*$/, '');
	};
}

function defined(value) {
	return !(typeof(value) === 'undefined' || value == null);
}
