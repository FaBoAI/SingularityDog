"""Injected human prompts only; no capture, files, or runtime approval.

``announce(text)``, ``write(text)`` and ``read()`` belong to the caller. Both
the displayed instructions and the audio finish before each input is read.
This helper does not replace the separate Enter-to-capture readiness prompt.
"""

CONFIRM_INSTRUCTION = (
    '確認した場合だけ y / yes / はい と入力して Enter。'
    '中止は n / no / q / いいえ。空Enterでは進みません。'
)
RETRY_MESSAGE = 'まだ確認されていません。確認または中止の返答を入力してください。'
AFFIRMATIVE = frozenset(('y', 'yes', 'はい'))
CANCEL = frozenset(('n', 'no', 'q', 'いいえ'))


def confirm(message, *, announce, read, write):
    """Return only for an explicit yes; retry blank or unknown text.

    An explicit no raises ValueError, matching the caller's existing abort
    handling. EOF, Ctrl+C, and callback failures propagate without retrying.
    No default confirmation or operator observation is created here.
    """
    if type(message) is not str or not message.strip():
        raise ValueError('A nonempty confirmation message is required')
    while True:
        write(message)
        write(CONFIRM_INSTRUCTION)
        announce(CONFIRM_INSTRUCTION)
        answer = read()
        if type(answer) is not str:
            raise TypeError('Confirmation input must be text')
        answer = answer.strip().lower()
        if answer in AFFIRMATIVE:
            return
        if answer in CANCEL:
            raise ValueError('Operator cancelled confirmation: ' + message)
        write(RETRY_MESSAGE)


PHYSICAL_NOTE_INSTRUCTION = (
    '実際に見た相対回転と復帰を、ご自分の言葉で入力して Enter。'
    '例（実際に観察した場合だけ）：指定した関節だけが回り、元の姿勢へ戻った。'
    '空欄や y / yes / はい / n / no / いいえ だけではメモになりません。'
    '中止は q / quit / 中止。'
)
PHYSICAL_NOTE_RETRY_MESSAGE = '観察メモはまだありません。実際に見た内容を入力してください。'
PHYSICAL_NOTE_CANCEL = frozenset(('q', 'quit', '中止'))
PHYSICAL_NOTE_CONFIRMATION_TOKENS = AFFIRMATIVE | CANCEL | frozenset(
    ('ok', 'okay', '了解', '確認済み'))


def physical_note(message, *, announce, read, write):
    """Return the user's note with surrounding whitespace removed, unchanged.

    Blank input and standalone confirmation tokens retry this prompt only.
    No observation, example text, or capture is generated on the user's behalf.
    Explicit cancellation raises ValueError; callback failures, EOF and Ctrl+C
    propagate to the caller's existing abort handling.
    """
    if type(message) is not str or not message.strip():
        raise ValueError('A nonempty physical-note message is required')
    while True:
        write(message)
        write(PHYSICAL_NOTE_INSTRUCTION)
        announce(PHYSICAL_NOTE_INSTRUCTION)
        answer = read()
        if type(answer) is not str:
            raise TypeError('Physical-note input must be text')
        note = answer.strip()
        token = note.lower()
        if token in PHYSICAL_NOTE_CANCEL:
            raise ValueError('Operator cancelled physical note: ' + message)
        if not note or token in PHYSICAL_NOTE_CONFIRMATION_TOKENS:
            write(PHYSICAL_NOTE_RETRY_MESSAGE)
            continue
        return note
