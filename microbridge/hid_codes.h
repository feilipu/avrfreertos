/*
Copyright 2015 Phillip Stevens

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.#include <string.h>
*/

#ifndef HID_CODES_H_
#define HID_CODES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <avr/pgmspace.h>



const uint8_t hid_lowercase[] PROGMEM = {
'\0',	//	No Event
'\0',	//	Overrun Error
'\0',	//	POST Fail
'\0',	//	ErrorUndefined
'a',	//	a A
'b',	//	b B
'c',	//	c C
'd',	//	d D
'e',	//	e E
'f',	//	f F
'g',	//	g G
'h',	//	h H
'i',	//	i I
'j',	//	j J
'k',	//	k K
'l',	//	l L
'm',	//	m M
'n',	//	n N
'o',	//	o O
'p',	//	p P
'q',	//	q Q
'r',	//	r R
's',	//	s S
't',	//	t T
'u',	//	u U
'v',	//	v V
'w',	//	w W
'x',	//	x X
'y',	//	y Y
'z',	//	z Z
'1',	//	1 !
'2',	//	2 @
'3',	//	3 #
'4',	//	4 $
'5',	//	5 %
'6',	//	6 ^
'7',	//	7 &
'8',	//	8 *
'9',	//	9 (
'0',	//	0 )
'\r',	//	Return
0x1b,	//	Escape
'\b',	//	Backspace
0x0b,	//	Tab
' ',	//	Space
'-',	//	- _
'=',	//	= +
'[',	//	[ {
']',	//	] }
'\\',	//	\ |
'\0',	//	Europe 1 (Note 2)
';',	//	; :
'\'',	//	' "
'`',	//	` ~
',',	//	, <
'.',	//	. >
'/',	//	/ ?
'\0',	//	Caps Lock
'\0',	//	F1
'\0',	//	F2
'\0',	//	F3
'\0',	//	F4
'\0',	//	F5
'\0',	//	F6
'\0',	//	F7
'\0',	//	F8
'\0',	//	F9
'\0',	//	F10
'\0',	//	F11
'\0',	//	F12
'\0',	//	Print Screen (Note 1)
'\0',	//	Scroll Lock
'\0',	//	Break (Ctrl-Pause)
'\0',	//	Pause
'\0',	//	Insert (Note 1)
'\0',	//	Home (Note 1)
'\0',	//	Page Up (Note 1)
0x7f,	//	Delete (Note 1)
'\0',	//	End (Note 1)
'\0',	//	Page Down (Note 1)
'\0',	//	Right Arrow (Note 1)
'\0',	//	Left Arrow (Note 1)
'\0',	//	Down Arrow (Note 1)
'\0',	//	Up Arrow (Note 1)
'\0',	//	Num Lock
'/',	//	Keypad / (Note 1)
'*',	//	Keypad *
'-',	//	Keypad -
'+',	//	Keypad +
'\r',	//	Keypad Enter
'1',	//	Keypad 1 End
'2',	//	Keypad 2 Down
'3',	//	Keypad 3 PageDn
'4',	//	Keypad 4 Left
'5',	//	Keypad 5
'6',	//	Keypad 6 Right
'7',	//	Keypad 7 Home
'8',	//	Keypad 8 Up
'9',	//	Keypad 9 PageUp
'0',	//	Keypad 0 Insert
'.',	//	Keypad . Delete
'\0',	//	Europe 2 (Note 2)
'\0',	//	App
'\0',	//	Keyboard Power
'=',	//	Keypad =
'\0',	//	F13
'\0',	//	F14
'\0',	//	F15
'\0',	//	F17
'\0',	//	F18
'\0',	//	F19
'\0',	//	F20
'\0',	//	F21
'\0',	//	F22
'\0',	//	F23
'\0',	//	F24
'\0',	//	Keyboard Execute
'\0',	//	Keyboard Help
'\0',	//	Keyboard Menu
'\0',	//	Keyboard Select
'\0',	//	Keyboard Stop
'\0',	//	Keyboard Again
'\0',	//	Keyboard Undo
'\0',	//	Keyboard Cut
'\0',	//	Keyboard Copy
'\0',	//	Keyboard Paste
'\0',	//	Keyboard Find
'\0',	//	Keyboard Mute
};

const uint8_t hid_uppercase[] PROGMEM = {
'\0',	//	No Event
'\0',	//	Overrun Error
'\0',	//	POST Fail
'\0',	//	ErrorUndefined
'A',	//	a A
'B',	//	b B
'C',	//	c C
'D',	//	d D
'E',	//	e E
'F',	//	f F
'G',	//	g G
'H',	//	h H
'I',	//	i I
'J',	//	j J
'K',	//	k K
'L',	//	l L
'M',	//	m M
'N',	//	n N
'O',	//	o O
'P',	//	p P
'Q',	//	q Q
'R',	//	r R
'S',	//	s S
'T',	//	t T
'U',	//	u U
'V',	//	v V
'W',	//	w W
'X',	//	x X
'Y',	//	y Y
'Z',	//	z Z
'!',	//	1 !
'@',	//	2 @
'#',	//	3 #
'$',	//	4 $
'%',	//	5 %
'^',	//	6 ^
'&',	//	7 &
'*',	//	8 *
'(',	//	9 (
')',	//	0 )
'\r',	//	Return
0x1b,	//	Escape
'\b',	//	Backspace
0x0b,	//	Tab
' ',	//	Space
'_',	//	- _
'+',	//	= +
'{',	//	[ {
'}',	//	] }
'|',	//	\ |
'\0',	//	Europe 1 (Note 2)
':',	//	; :
'\"',	//	' "
'~',	//	` ~
'<',	//	, <
'>',	//	. >
'\?',	//	/ ?
'\0',	//	Caps Lock
'\0',	//	F1
'\0',	//	F2
'\0',	//	F3
'\0',	//	F4
'\0',	//	F5
'\0',	//	F6
'\0',	//	F7
'\0',	//	F8
'\0',	//	F9
'\0',	//	F10
'\0',	//	F11
'\0',	//	F12
'\0',	//	Print Screen (Note 1)
'\0',	//	Scroll Lock
'\0',	//	Break (Ctrl-Pause)
'\0',	//	Pause
'\0',	//	Insert (Note 1)
'\0',	//	Home (Note 1)
'\0',	//	Page Up (Note 1)
0x7f,	//	Delete (Note 1)
'\0',	//	End (Note 1)
'\0',	//	Page Down (Note 1)
'\0',	//	Right Arrow (Note 1)
'\0',	//	Left Arrow (Note 1)
'\0',	//	Down Arrow (Note 1)
'\0',	//	Up Arrow (Note 1)
'\0',	//	Num Lock
'/',	//	Keypad / (Note 1)
'*',	//	Keypad *
'-',	//	Keypad -
'+',	//	Keypad +
'\r',	//	Keypad Enter
'1',	//	Keypad 1 End
'2',	//	Keypad 2 Down
'3',	//	Keypad 3 PageDn
'4',	//	Keypad 4 Left
'5',	//	Keypad 5
'6',	//	Keypad 6 Right
'7',	//	Keypad 7 Home
'8',	//	Keypad 8 Up
'9',	//	Keypad 9 PageUp
'0',	//	Keypad 0 Insert
'.',	//	Keypad . Delete
'\0',	//	Europe 2 (Note 2)
'\0',	//	App
'\0',	//	Keyboard Power
'=',	//	Keypad =
'\0',	//	F13
'\0',	//	F14
'\0',	//	F15
'\0',	//	F17
'\0',	//	F18
'\0',	//	F19
'\0',	//	F20
'\0',	//	F21
'\0',	//	F22
'\0',	//	F23
'\0',	//	F24
'\0',	//	Keyboard Execute
'\0',	//	Keyboard Help
'\0',	//	Keyboard Menu
'\0',	//	Keyboard Select
'\0',	//	Keyboard Stop
'\0',	//	Keyboard Again
'\0',	//	Keyboard Undo
'\0',	//	Keyboard Cut
'\0',	//	Keyboard Copy
'\0',	//	Keyboard Paste
'\0',	//	Keyboard Find
'\0',	//	Keyboard Mute
};




#ifdef __cplusplus
}
#endif

#endif /* HID_CODES_H_ */
