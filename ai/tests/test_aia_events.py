from __future__ import annotations

import unittest
from unittest.mock import patch

from llm_cli_chat.aia_events import AiaMessageLineSender, send_aia_message_event


class AiaEventTests(unittest.TestCase):
    def test_send_aia_message_event_posts_completed_message(self) -> None:
        with patch("llm_cli_chat.aia_events._post_json", return_value={"ok": True}) as post_json:
            response = self._run(
                send_aia_message_event("http://ground", " Completed response. "),
            )

        self.assertEqual(response, {"ok": True})
        post_json.assert_called_once_with(
            "http://ground/aia/messages",
            {
                "message": "Completed response.",
                "source": "aia",
                "target": "hololens",
            },
            2.0,
        )

    def test_send_aia_message_event_skips_empty_messages(self) -> None:
        with patch("llm_cli_chat.aia_events._post_json") as post_json:
            response = self._run(send_aia_message_event("http://ground", " "))

        self.assertIsNone(response)
        post_json.assert_not_called()

    def test_line_sender_posts_completed_lines_only(self) -> None:
        sender = AiaMessageLineSender("http://ground")

        with patch("llm_cli_chat.aia_events._post_json", return_value={"ok": True}) as post_json:
            self.assertEqual(self._run(sender.send_completed_lines("I will check.")), 0)
            self.assertEqual(self._run(sender.send_completed_lines("I will check.\nChecking")), 1)
            self._run(
                sender.send_completed_lines(
                    "I will check.\nChecking current biometrics...\n\nHeart rate is 82.",
                )
            )

        self.assertEqual(post_json.call_count, 2)
        self.assertEqual(
            post_json.call_args_list[0].args[1]["message"],
            "I will check.",
        )
        self.assertEqual(
            post_json.call_args_list[1].args[1]["message"],
            "Checking current biometrics...",
        )

    def test_line_sender_posts_remaining_tail_on_completion(self) -> None:
        sender = AiaMessageLineSender("http://ground")

        with patch("llm_cli_chat.aia_events._post_json", return_value={"ok": True}) as post_json:
            self._run(sender.send_completed_lines("Checking current biometrics...\nHeart"))
            self._run(sender.send_remaining("Checking current biometrics...\nHeart rate is 82."))

        self.assertEqual(post_json.call_count, 2)
        self.assertEqual(
            post_json.call_args_list[0].args[1]["message"],
            "Checking current biometrics...",
        )
        self.assertEqual(
            post_json.call_args_list[1].args[1]["message"],
            "Heart rate is 82.",
        )

    def test_line_sender_does_not_advance_after_failed_post(self) -> None:
        sender = AiaMessageLineSender("http://ground")

        with patch(
            "llm_cli_chat.aia_events._post_json",
            side_effect=[RuntimeError("offline"), {"ok": True}],
        ) as post_json:
            with self.assertRaises(RuntimeError):
                self._run(sender.send_completed_lines("Checking current biometrics...\n"))
            self._run(sender.send_remaining("Checking current biometrics...\nHeart rate is 82."))

        self.assertEqual(post_json.call_count, 2)
        self.assertEqual(
            post_json.call_args_list[1].args[1]["message"],
            "Checking current biometrics...\nHeart rate is 82.",
        )

    def _run(self, coroutine):
        import asyncio

        return asyncio.run(coroutine)


if __name__ == "__main__":
    unittest.main()
