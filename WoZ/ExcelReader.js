// ============================================================================
// ExcelReader.js
// ============================================================================
//
//  Created by leuski on 5/6/15.
//  Copyright (c) 2015 Anton Leuski and ICT. All rights reserved.
// ============================================================================

function ExcelReader(usingHeaderRow) {
	this.usingHeaderRow = usingHeaderRow;
}

ExcelReader.cell_as_string = function(object, address, blank_value) {
	var cell_address = XLS.utils.encode_cell(address);
	var value = object[cell_address];
	if (!defined(value)) {
		return undefined;
	}
	value = value.w;
	if (!defined(value)) {
		return undefined;
	}
	if (value == "") {
		return arguments.length > 1 ? blank_value : "";
	}
	return value;
};

ExcelReader.cell_as_number = function (object, address) {
	var cell_address = XLS.utils.encode_cell(address);
	var value = object[cell_address];
	if (!defined(value)) {
		return undefined;
	}
	value = value.w;
	if (!defined(value)) {
		return undefined;
	}
	if (value == "") {
		return undefined;
	}
	value = Number(value);
	if (isNaN(value)) {
		return undefined;
	}
	return value;
};

ExcelReader.find_or_make_object_with_id = function (inObject, id) {
	if (!defined(id)) { return undefined; }

	var object = inObject[id];
	if (!defined(object)) {
		object = {};
		inObject[id] = object;
	}

	return object;
};

ExcelReader.find_object_with_id = function (inObject, id) {
	if (!defined(id)) { return undefined; }
	return inObject[id];
};

ExcelReader.prototype._read_button_indexes_from_sheet = function (sheet, range, R) {
	this.buttonIndexes = {};
	this.idColumnIndex = -1;
	for(var C = range.s.c; C <= range.e.c; ++C) {
		var value = ExcelReader.cell_as_string(sheet, {c:C, r:R});
		if (value == undefined) {
			continue;
		}
		value = value.toLowerCase();
		if (value == 'answer') {
			value = 'woz.tooltip';
		}
		if (!value.startsWith('woz.')) {
			continue;
		}
		value = value.substring('woz.'.length);
		this.buttonIndexes[C] = value;
		if (value == 'id') {
			this.idColumnIndex = C;
		}
	}
};

ExcelReader.prototype._read_button_from_row = function (model, sheet, range, R) {
	var button = ExcelReader.find_or_make_object_with_id(model.buttons, ExcelReader.cell_as_string(sheet, {c:this.idColumnIndex, r:R}, undefined));
	if (button == undefined) {
		return model;
	}

	button.label = "";
	button.tooltip = "";

	for(var C = range.s.c; C <= range.e.c; ++C) {
		var content = ExcelReader.cell_as_string(sheet, {c:C, r:R});
		if (content == undefined) {
			continue;
		}
		var key = this.buttonIndexes[C];
		if (key == undefined || key == 'id') {
			continue;
		}

		if (key == 'transition') {
			if (!defined(button.transitions)) {
				button.transitions = {};
			}
			button.transitions["_any"] = content;
		} else if (key.startsWith("badge.")) {
			if (!defined(button.badges)) {
				button.badges = {};
			}
			button.badges[key.substring("badge.".length)] = content;
		} else {
			button[key] = content;
		}
	}

	return model;
};

ExcelReader.prototype._read_buttons_from_sheet = function (model, sheet) {
	if (!defined(sheet)) {
		return model;
	}

	var range = XLS.utils.decode_range(sheet['!ref']);
	for(var R = range.s.r; R <= range.e.r; ++R) {
		if (R == range.s.r && this.usingHeaderRow) {
			this._read_button_indexes_from_sheet(sheet, range, R)
		} else {
			model = this._read_button_from_row(model, sheet, range, R)
		}
	}

	return model;
};

ExcelReader.prototype._read_rows_from_sheet = function (model, sheet) {
	if (!defined(sheet)) {
		return model;
	}

	var range = XLS.utils.decode_range(sheet['!ref']);
	for(var R = range.s.r; R <= range.e.r; ++R) {
		var theColumn = null;
		for(var C = range.s.c; C <= range.e.c; ++C) {
			var content = ExcelReader.cell_as_string(sheet, {c:C, r:R});

			if (C-range.s.c == 0) {
				theColumn = ExcelReader.find_or_make_object_with_id(model.rows, content);
				if (theColumn == undefined) { break; }
				theColumn.label = "";
				theColumn.buttons = [];
			} else if (C-range.s.c == 1) {
				theColumn.label = defined(content) ? content : "";
			} else {
				var theButton = ExcelReader.find_object_with_id(model.buttons, content);
				if (theButton == undefined) { continue; }
				theColumn.buttons.push(content);
			}

		}
	}

	return model;
};

ExcelReader.prototype._read_screens_from_sheet = function (model, sheet) {
	if (!defined(sheet)) {
		return model;
	}

	var allStates = {};
	var range = XLS.utils.decode_range(sheet['!ref']);

	for(var R = range.s.r; R <= range.e.r; ++R) {
		for (var C = range.s.c; C <= range.e.c; ++C) {
			var content = ExcelReader.cell_as_string(sheet, {c: C, r: R});
			var theState;
			if (R-range.s.r == 0) {
				theState = ExcelReader.find_or_make_object_with_id(model.screens, content);
				if (theState == undefined) { continue; }
				theState.label = "";
				theState.rows = [];
				allStates[C] = theState;
			} else {
				theState = allStates[C];
				if (!defined(theState)) { continue; }
				if (R-range.s.r == 1) {
					theState.label = content;
				} else {
					var theColumn = ExcelReader.find_object_with_id(model.rows, content);
					if (theColumn == undefined) { continue; }
					theState.rows.push(content);
				}
			}
		}
	}

	return model;
};

ExcelReader.prototype._read_colors_from_sheet = function (model, sheet) {

	var _get_color_components = function (sheet, R, columnIndexes) {
		if (arguments.length != 6) { return undefined; }
		var comp = [];
		for(var i = 0; i < 3; ++i) {
			var ci = columnIndexes[arguments[3+i]];
			if (!defined(ci)) { return undefined; }
			var value = ExcelReader.cell_as_number(sheet, {c: ci, r: R});
			if (!defined(value)) { return undefined; }
			comp[i] = value;
		}
		return comp;
	};

	var _hsb2rgb = function (comp) {
		if (comp == undefined) { return undefined; }

		if (comp[1] == 0) {
			return [comp[2], comp[2], comp[2]];
		}

		var hue2rgb = function hue2rgb(p, q, t) {
			if(t < 0) t += 1;
			if(t > 1) t -= 1;
			if(t < 1/6) return p + (q - p) * 6 * t;
			if(t < 1/2) return q;
			if(t < 2/3) return p + (q - p) * (2/3 - t) * 6;
			return p;
		};

		var q = comp[2] < 0.5 ? comp[2] * (1 + comp[1]) : comp[2] + comp[1] - comp[2] * comp[1];
		var p = 2 * comp[2] - q;
		return [ hue2rgb(p, q, comp[0] + 1/3), hue2rgb(p, q, comp[0]), hue2rgb(p, q, comp[0] - 1/3) ];
	};

	var _rgb_from_number = function (n) {
		if (n < 0) return 0;
		if (n > 1) return n;
		return Math.min(Math.floor(255*n), 255);
	};

	var _set_color_components = function (theColor, comp) {
		theColor.r = _rgb_from_number(comp[0]);
		theColor.g = _rgb_from_number(comp[1]);
		theColor.b = _rgb_from_number(comp[2]);
		return theColor;
	};

	if (!defined(sheet)) {
		return model;
	}

	var range = XLS.utils.decode_range(sheet['!ref']);
	var columnIndexes = {};
	var useRGB = false;

	for(var R = range.s.r; R <= range.e.r; ++R) {
		if (R == range.s.r) {
			for (var C = range.s.c; C <= range.e.c; ++C) {
				var content = ExcelReader.cell_as_string(sheet, {c: C, r: R});
				if (content == undefined) { continue; }
				content = content.substring(0,1).toLowerCase();
				columnIndexes[content] = C;
			}
			if (defined(columnIndexes["h"]) && defined(columnIndexes["s"]) && defined(columnIndexes["b"]) && defined(columnIndexes["i"])) {
				useRGB = false;
			} else if (defined(columnIndexes["r"]) && defined(columnIndexes["g"]) && defined(columnIndexes["b"]) && defined(columnIndexes["i"])) {
				useRGB = true;
			} else {
				return model;
			}
		} else {
			var index = ExcelReader.cell_as_string(sheet, {c: columnIndexes.i, r: R}, undefined);
			if (index == undefined) { continue; }

			var comp;

			if (useRGB) {
				comp = _get_color_components(sheet, R, columnIndexes, "r", "g", "b");
			} else {
				comp = _get_color_components(sheet, R, columnIndexes, "h", "s", "b");
				comp = _hsb2rgb(comp);
			}

			if (comp == undefined) { continue; }

			var theColor = ExcelReader.find_or_make_object_with_id(model.colors, index);
			if (theColor == undefined) { continue; }

			theColor = _set_color_components(theColor, comp);
		}
	}

	return model;
};


ExcelReader.prototype.convert_workbook_to_json = function (workbook) {
	var model = {colors:{}, buttons:{}, rows:{}, screens:{}};
	model = this._read_colors_from_sheet(model, workbook.Sheets['colors']);
	model = this._read_buttons_from_sheet(model, workbook.Sheets['buttons']);
	model = this._read_rows_from_sheet(model, workbook.Sheets['rows']);
	model = this._read_screens_from_sheet(model, workbook.Sheets['screens']);
	return model;
};
