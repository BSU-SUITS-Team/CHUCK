from __future__ import annotations

import unittest
from unittest.mock import patch

from llm_cli_chat.aia_events import send_aia_message_event


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

    def _run(self, coroutine):
        import asyncio

        return asyncio.run(coroutine)


if __name__ == "__main__":
    unittest.main()
