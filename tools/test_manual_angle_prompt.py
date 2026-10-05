"""Pure injected prompt tests; no hardware, subprocesses, or capture files."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'manual_angle_prompt', Path(__file__).with_name('manual_angle_prompt.py'))
prompt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prompt)


class Interaction:
    def __init__(self, answers):
        self.answers = iter(answers)
        self.events = []

    def write(self, text):
        self.events.append(('write', text))

    def announce(self, text):
        self.events.append(('audio', text))

    def read(self):
        self.events.append(('read', None))
        answer = next(self.answers)
        if isinstance(answer, BaseException):
            raise answer
        return answer

    def confirm(self):
        return prompt.confirm('Physical observation required',
            announce=self.announce, read=self.read, write=self.write)

    def physical_note(self):
        return prompt.physical_note('Actual physical observation note required',
            announce=self.announce, read=self.read, write=self.write)

    def reads(self):
        return sum(event[0] == 'read' for event in self.events)


class ManualAnglePromptTests(unittest.TestCase):
    def test_only_explicit_affirmative_tokens_return(self):
        for answer in ('y', 'yes', 'はい', ' Y ', ' YES\n', ' はい '):
            with self.subTest(answer=answer):
                interaction = Interaction([answer])
                self.assertIsNone(interaction.confirm())
                self.assertEqual(interaction.reads(), 1)

    def test_empty_enter_retries_until_explicit_yes(self):
        interaction = Interaction(['', '', 'y'])
        interaction.confirm()
        self.assertEqual(interaction.reads(), 3)
        self.assertEqual(interaction.events.count(('write', prompt.RETRY_MESSAGE)), 2)

    def test_whitespace_enter_is_not_confirmation(self):
        interaction = Interaction([' \t\n', 'yes'])
        interaction.confirm()
        self.assertEqual(interaction.reads(), 2)

    def test_unknown_tokens_never_confirm(self):
        for answer in ('maybe', '1', 'true', 'ok', 'yes!', '確認', 'はいです'):
            with self.subTest(answer=answer):
                interaction = Interaction([answer, 'n'])
                with self.assertRaisesRegex(ValueError, 'Operator cancelled'):
                    interaction.confirm()
                self.assertEqual(interaction.reads(), 2)

    def test_unknown_answer_can_be_followed_by_explicit_yes(self):
        interaction = Interaction(['unknown', 'はい'])
        interaction.confirm()
        self.assertEqual(interaction.reads(), 2)

    def test_each_explicit_cancel_aborts_without_another_read(self):
        for answer in ('n', 'no', 'q', 'いいえ', ' N ', ' NO\n', ' Q '):
            with self.subTest(answer=answer):
                interaction = Interaction([answer, 'y'])
                with self.assertRaisesRegex(ValueError, 'Operator cancelled'):
                    interaction.confirm()
                self.assertEqual(interaction.reads(), 1)

    def test_empty_then_cancel_aborts(self):
        interaction = Interaction(['', 'no', 'y'])
        with self.assertRaises(ValueError):
            interaction.confirm()
        self.assertEqual(interaction.reads(), 2)

    def test_eof_aborts_without_retry(self):
        interaction = Interaction([EOFError(), 'y'])
        with self.assertRaises(EOFError):
            interaction.confirm()
        self.assertEqual(interaction.reads(), 1)

    def test_interrupt_aborts_without_retry(self):
        interaction = Interaction([KeyboardInterrupt(), 'y'])
        with self.assertRaises(KeyboardInterrupt):
            interaction.confirm()
        self.assertEqual(interaction.reads(), 1)

    def test_eof_after_empty_answer_still_aborts(self):
        interaction = Interaction(['', EOFError(), 'y'])
        with self.assertRaises(EOFError):
            interaction.confirm()
        self.assertEqual(interaction.reads(), 2)

    def test_interrupt_after_unknown_answer_still_aborts(self):
        interaction = Interaction(['unknown', KeyboardInterrupt(), 'y'])
        with self.assertRaises(KeyboardInterrupt):
            interaction.confirm()
        self.assertEqual(interaction.reads(), 2)

    def test_message_and_audio_precede_every_input_including_retries(self):
        interaction = Interaction(['', 'unknown', 'y'])
        interaction.confirm()
        attempt = [('write', 'Physical observation required'),
            ('write', prompt.CONFIRM_INSTRUCTION), ('audio', prompt.CONFIRM_INSTRUCTION),
            ('read', None)]
        retry = [('write', prompt.RETRY_MESSAGE)]
        self.assertEqual(interaction.events, attempt + retry + attempt + retry + attempt)

    def test_audio_failure_propagates_before_reading(self):
        interaction = Interaction(['y'])
        def failed_audio(text):
            raise OSError('audio failed')
        with self.assertRaisesRegex(OSError, 'audio failed'):
            prompt.confirm('Physical observation required', announce=failed_audio,
                read=interaction.read, write=interaction.write)
        self.assertEqual(interaction.reads(), 0)

    def test_display_failure_propagates_before_audio_or_input(self):
        interaction = Interaction(['y'])
        def failed_write(text):
            raise OSError('display failed')
        with self.assertRaisesRegex(OSError, 'display failed'):
            prompt.confirm('Physical observation required', announce=interaction.announce,
                read=interaction.read, write=failed_write)
        self.assertEqual(interaction.events, [])

    def test_nontext_answers_do_not_confirm(self):
        for answer in (None, True, 1, b'y'):
            with self.subTest(answer=answer):
                interaction = Interaction([answer, 'y'])
                with self.assertRaises(TypeError):
                    interaction.confirm()
                self.assertEqual(interaction.reads(), 1)

    def test_invalid_message_does_not_prompt(self):
        for message in (None, '', ' \n', True):
            with self.subTest(message=message):
                interaction = Interaction(['y'])
                with self.assertRaises(ValueError):
                    prompt.confirm(message, announce=interaction.announce,
                        read=interaction.read, write=interaction.write)
                self.assertEqual(interaction.events, [])

    def test_all_io_is_injected(self):
        interaction = Interaction(['', 'y'])
        with (patch('builtins.input', side_effect=AssertionError('Implicit input')),
              patch('builtins.print', side_effect=AssertionError('Implicit output')),
              patch('builtins.open', side_effect=AssertionError('Implicit file access'))):
            interaction.confirm()
        self.assertEqual(interaction.reads(), 2)


class PhysicalNoteTests(unittest.TestCase):
    def test_returns_only_the_users_text_without_completion_or_translation(self):
        for text in ('足先を下げて戻した', '  Saw a small relative turn; returned.  ',
                     '\n下脚だけを下げた。\n同じ位置へ戻った。\n'):
            with self.subTest(text=text):
                interaction = Interaction([text])
                self.assertEqual(interaction.physical_note(), text.strip())
                self.assertEqual(interaction.reads(), 1)

    def test_empty_input_retries_without_inventing_a_note(self):
        interaction = Interaction(['', '', '上脚を顔側へ傾けて戻した'])
        self.assertEqual(interaction.physical_note(), '上脚を顔側へ傾けて戻した')
        self.assertEqual(interaction.reads(), 3)
        self.assertEqual(interaction.events.count(('write', prompt.PHYSICAL_NOTE_RETRY_MESSAGE)), 2)

    def test_whitespace_input_retries(self):
        interaction = Interaction([' \t\n', '相対角の回転と復帰を見た'])
        self.assertEqual(interaction.physical_note(), '相対角の回転と復帰を見た')
        self.assertEqual(interaction.reads(), 2)

    def test_standalone_confirmation_tokens_are_not_notes(self):
        for token in ('y', 'yes', 'はい', 'n', 'no', 'いいえ', ' Y ', ' YES\n', ' NO ',
                      'OK', 'okay', '了解', '確認済み', ' OKAY\n', ' 了解 '):
            with self.subTest(token=token):
                interaction = Interaction([token, '下脚だけが動いて元へ戻った'])
                self.assertEqual(interaction.physical_note(), '下脚だけが動いて元へ戻った')
                self.assertEqual(interaction.reads(), 2)

    def test_confirmation_token_then_cancel_does_not_create_a_note(self):
        interaction = Interaction(['はい', 'q', '足先を下げた'])
        with self.assertRaisesRegex(ValueError, 'Operator cancelled physical note'):
            interaction.physical_note()
        self.assertEqual(interaction.reads(), 2)

    def test_explicit_note_cancellation_aborts_without_another_read(self):
        for token in ('q', 'quit', '中止', ' Q ', ' QUIT\n', ' 中止 '):
            with self.subTest(token=token):
                interaction = Interaction([token, '足先を下げた'])
                with self.assertRaisesRegex(ValueError, 'Operator cancelled physical note'):
                    interaction.physical_note()
                self.assertEqual(interaction.reads(), 1)

    def test_eof_aborts_without_retry(self):
        interaction = Interaction([EOFError(), '足先を下げた'])
        with self.assertRaises(EOFError):
            interaction.physical_note()
        self.assertEqual(interaction.reads(), 1)

    def test_interrupt_aborts_without_retry(self):
        interaction = Interaction([KeyboardInterrupt(), '足先を下げた'])
        with self.assertRaises(KeyboardInterrupt):
            interaction.physical_note()
        self.assertEqual(interaction.reads(), 1)

    def test_eof_after_empty_input_aborts(self):
        interaction = Interaction(['', EOFError(), '足先を下げた'])
        with self.assertRaises(EOFError):
            interaction.physical_note()
        self.assertEqual(interaction.reads(), 2)

    def test_interrupt_after_confirmation_token_aborts(self):
        interaction = Interaction(['yes', KeyboardInterrupt(), '足先を下げた'])
        with self.assertRaises(KeyboardInterrupt):
            interaction.physical_note()
        self.assertEqual(interaction.reads(), 2)

    def test_example_and_response_instructions_precede_every_input(self):
        interaction = Interaction(['', 'yes', '膝の相対角を保って戻した'])
        interaction.physical_note()
        attempt = [('write', 'Actual physical observation note required'),
            ('write', prompt.PHYSICAL_NOTE_INSTRUCTION), ('audio', prompt.PHYSICAL_NOTE_INSTRUCTION),
            ('read', None)]
        retry = [('write', prompt.PHYSICAL_NOTE_RETRY_MESSAGE)]
        self.assertEqual(interaction.events, attempt + retry + attempt + retry + attempt)
        self.assertIn('例', prompt.PHYSICAL_NOTE_INSTRUCTION)
        self.assertIn('Enter', prompt.PHYSICAL_NOTE_INSTRUCTION)
        self.assertIn('実際に観察した場合だけ', prompt.PHYSICAL_NOTE_INSTRUCTION)
        self.assertIn('指定した関節だけが回り、元の姿勢へ戻った', prompt.PHYSICAL_NOTE_INSTRUCTION)
        self.assertNotIn('足先を下げ', prompt.PHYSICAL_NOTE_INSTRUCTION)

    def test_audio_failure_propagates_before_input(self):
        interaction = Interaction(['足先を下げた'])
        def failed_audio(text):
            raise OSError('audio failed')
        with self.assertRaisesRegex(OSError, 'audio failed'):
            prompt.physical_note('Actual note required', announce=failed_audio,
                read=interaction.read, write=interaction.write)
        self.assertEqual(interaction.reads(), 0)

    def test_display_failure_propagates_before_audio_or_input(self):
        interaction = Interaction(['足先を下げた'])
        def failed_write(text):
            raise OSError('display failed')
        with self.assertRaisesRegex(OSError, 'display failed'):
            prompt.physical_note('Actual note required', announce=interaction.announce,
                read=interaction.read, write=failed_write)
        self.assertEqual(interaction.events, [])

    def test_nontext_input_does_not_become_a_note(self):
        for answer in (None, True, 1, b'saw a turn'):
            with self.subTest(answer=answer):
                interaction = Interaction([answer, '足先を下げた'])
                with self.assertRaises(TypeError):
                    interaction.physical_note()
                self.assertEqual(interaction.reads(), 1)

    def test_invalid_message_does_not_prompt(self):
        for message in (None, '', ' \n', True):
            with self.subTest(message=message):
                interaction = Interaction(['足先を下げた'])
                with self.assertRaises(ValueError):
                    prompt.physical_note(message, announce=interaction.announce,
                        read=interaction.read, write=interaction.write)
                self.assertEqual(interaction.events, [])

    def test_retry_uses_only_injected_prompt_io(self):
        interaction = Interaction(['', 'n', '下脚だけを下げて戻した'])
        with (patch('builtins.input', side_effect=AssertionError('Implicit input')),
              patch('builtins.print', side_effect=AssertionError('Implicit output')),
              patch('builtins.open', side_effect=AssertionError('Implicit file access'))):
            self.assertEqual(interaction.physical_note(), '下脚だけを下げて戻した')
        self.assertEqual(interaction.reads(), 3)

    def test_confirmation_words_inside_an_actual_sentence_are_preserved(self):
        for text in ('はい、下脚だけを下げて戻した', 'No other relative joint moved.',
                     '中止せず、足先を下げて戻した', '確認済み：指定した関節だけが回った'):
            with self.subTest(text=text):
                self.assertEqual(Interaction([text]).physical_note(), text)


if __name__ == '__main__':
    unittest.main()
