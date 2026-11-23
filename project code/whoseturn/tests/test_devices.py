import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from whoseturn.devices import Head, open_client
from host.robot_test import ProtocolError

class AdapterTests(unittest.TestCase):
    def test_rejects_old_firmware_before_establishing_session(self):
        transport, client = Mock(), Mock()
        client.request.return_value = {"state": "FAULT", "wiring_version": "0.8"}
        with patch("whoseturn.devices.SerialTransport", return_value=transport), \
             patch("whoseturn.devices.Client", return_value=client), \
             patch("whoseturn.devices.time.sleep"):
            with self.assertRaises(ProtocolError):
                open_client("hardware", "unused-test-port")
        client.hello.assert_not_called()
        transport.close.assert_called_once()

    def test_rejects_active_head_firmware_without_clearing_or_stopping(self):
        transport, client = Mock(), Mock()
        client.request.return_value = {"state": "ARMED", "protocol_version": "head-1"}
        with patch("whoseturn.devices.SerialTransport", return_value=transport), \
             patch("whoseturn.devices.Client", return_value=client), \
             patch("whoseturn.devices.time.sleep"):
            with self.assertRaises(ProtocolError):
                open_client("hardware", "unused-test-port")
        client.request.assert_called_once_with("status")
        client.hello.assert_not_called()
        transport.close.assert_called_once()

    def test_head_prepare_checks_status_before_sending_neutral_pulse(self):
        cfg = SimpleNamespace(neutral_us=1500, tilt_us=1500)
        for status, allowed in [
            ({"protocol_version": "head-1", "state": "ARMED", "mode": "head"}, True),
            ({"protocol_version": "head-1", "state": "FAULT", "mode": "head"}, False),
            ({"state": "ARMED", "mode": "head"}, False),
        ]:
            with self.subTest(status=status):
                client = Mock(epoch=3)
                client.request.return_value = status
                head = Head(cfg, Mock(), 0, client)
                if allowed:
                    head.prepare()
                    self.assertTrue(head.armed)
                    client.request.assert_any_call("servo", pan_us=1500, tilt_us=1500, arm_epoch=3)
                else:
                    with self.assertRaises(ProtocolError):
                        head.prepare()
                    self.assertFalse(head.armed)
                    self.assertFalse(any(c.args[0] == "servo" for c in client.request.call_args_list))

if __name__ == "__main__":
    unittest.main()
