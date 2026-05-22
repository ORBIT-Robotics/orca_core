"""Shared Dynamixel status/reboot preflight helpers for ORCA hand scripts."""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time
from typing import Mapping, Sequence

from orca_core.utils.yaml_io import read_yaml


ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
DYNAMIXEL_REBOOT_SETTLE_SEC = 1.5


@dataclass(frozen=True)
class DynamixelMotorStatus:
    motor_id: int
    hardware_error_status: int | None
    comm_result: int | None
    dxl_error: int | None
    packet_status: str

    @property
    def ok(self) -> bool:
        return (
            self.hardware_error_status == 0
            and self.comm_result == 0
            and (self.dxl_error or 0) == 0
        )

    def summary(self) -> str:
        if self.hardware_error_status is None:
            hw_text = "Hardware Error Status(70)=????"
        else:
            hw_text = f"Hardware Error Status(70)=0x{self.hardware_error_status:02x}"
        comm_text = "comm_result=????" if self.comm_result is None else f"comm_result={self.comm_result}"
        dxl_text = "dxl_error=????" if self.dxl_error is None else f"dxl_error=0x{self.dxl_error:02x}"
        return (
            f"id={self.motor_id:02d} {hw_text}, {comm_text}, {dxl_text}, "
            f"packet_status={self.packet_status}"
        )


@dataclass(frozen=True)
class DynamixelRoleStatusReport:
    role: str
    port: str
    baudrate: int
    statuses: tuple[DynamixelMotorStatus, ...] = ()
    transport_error: str | None = None

    @property
    def ok(self) -> bool:
        return self.transport_error is None and bool(self.statuses) and all(
            status.ok for status in self.statuses
        )

    @property
    def can_attempt_reboot(self) -> bool:
        return self.transport_error is None and bool(self.statuses)

    @property
    def bad_motor_ids(self) -> list[int]:
        return [status.motor_id for status in self.statuses if not status.ok]

    def problem_summaries(self) -> list[str]:
        if self.transport_error is not None:
            return [
                f"[{self.role}] transport_error={self.transport_error}, "
                f"port={self.port}, baudrate={self.baudrate}"
            ]
        return [
            f"[{self.role}] {status.summary()}"
            for status in self.statuses
            if not status.ok
        ]


def _load_model_config(spec) -> dict:
    config = read_yaml(str(spec.model_path / "config.yaml"))
    if not isinstance(config, dict):
        raise ValueError(f"{spec.role} model config must be a mapping.")
    return config


def _role_motor_type(spec) -> str:
    config = _load_model_config(spec)
    return str(config.get("motor_type", "dynamixel")).strip().lower()


def _role_motor_ids(spec) -> list[int]:
    config = _load_model_config(spec)
    motor_ids = config.get("motor_ids")
    if not isinstance(motor_ids, list) or not motor_ids:
        raise ValueError(
            f"{spec.role} model config must define a non-empty motor_ids list."
        )
    disabled_motor_ids = config.get("disabled_motor_ids", [])
    if disabled_motor_ids is None:
        disabled_motor_ids = []
    if not isinstance(disabled_motor_ids, list):
        raise ValueError(f"{spec.role} disabled_motor_ids must be a list when provided.")
    disabled_set = {int(motor_id) for motor_id in disabled_motor_ids}
    return [int(motor_id) for motor_id in motor_ids if int(motor_id) not in disabled_set]


def _normalize_motor_ids(spec, motor_ids: Sequence[int] | None) -> list[int]:
    configured = _role_motor_ids(spec)
    if motor_ids is None:
        return configured
    requested = [int(motor_id) for motor_id in motor_ids]
    configured_set = set(configured)
    unknown = [motor_id for motor_id in requested if motor_id not in configured_set]
    if unknown:
        raise ValueError(
            f"{spec.role} requested unknown motor IDs {unknown}; configured IDs are {configured}."
        )
    return requested


def _format_packet_status(packet_handler, comm_result: int, dxl_error: int) -> str:
    parts = [f"comm_result={int(comm_result)}", f"dxl_error=0x{int(dxl_error):02x}"]
    if comm_result != packet_handler.dxl.COMM_SUCCESS:
        parts.append(packet_handler.getTxRxResult(comm_result))
    if dxl_error:
        parts.append(packet_handler.getRxPacketError(dxl_error))
    if len(parts) == 2:
        parts.append("ok")
    return "; ".join(parts)


def _read_hardware_error_statuses(
    packet_handler,
    port_handler,
    motor_ids: Sequence[int],
) -> tuple[DynamixelMotorStatus, ...]:
    statuses: list[DynamixelMotorStatus] = []
    for motor_id in motor_ids:
        value, comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler,
            int(motor_id),
            ADDR_HARDWARE_ERROR_STATUS,
        )
        packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
        if comm_result != packet_handler.dxl.COMM_SUCCESS:
            status = DynamixelMotorStatus(
                motor_id=int(motor_id),
                hardware_error_status=None,
                comm_result=int(comm_result),
                dxl_error=int(dxl_error),
                packet_status=packet_status,
            )
            print(f"    {status.summary()}")
            statuses.append(status)
            continue
        status = DynamixelMotorStatus(
            motor_id=int(motor_id),
            hardware_error_status=int(value),
            comm_result=int(comm_result),
            dxl_error=int(dxl_error),
            packet_status=packet_status,
        )
        print(f"    {status.summary()}")
        statuses.append(status)
    return tuple(statuses)


def _read_hardware_errors(packet_handler, port_handler, motor_ids: list[int]) -> dict[int, int | None]:
    statuses = _read_hardware_error_statuses(packet_handler, port_handler, motor_ids)
    return {
        status.motor_id: status.hardware_error_status
        for status in statuses
    }


def _make_transport_error_report(spec, message: str) -> DynamixelRoleStatusReport:
    return DynamixelRoleStatusReport(
        role=spec.role,
        port=spec.port,
        baudrate=spec.baudrate,
        transport_error=message,
    )


def check_dynamixel_role_status(
    spec,
    *,
    motor_ids: Sequence[int] | None = None,
    samples: int = 3,
    sample_interval_sec: float = 0.1,
) -> DynamixelRoleStatusReport:
    try:
        import dynamixel_sdk
    except ModuleNotFoundError:
        return _make_transport_error_report(
            spec,
            "dynamixel_sdk is missing. Run this from the control conda env.",
        )

    selected_motor_ids = _normalize_motor_ids(spec, motor_ids)
    port_handler = dynamixel_sdk.PortHandler(spec.port)
    packet_handler = dynamixel_sdk.PacketHandler(2.0)
    packet_handler.dxl = dynamixel_sdk

    if not port_handler.openPort():
        return _make_transport_error_report(spec, f"failed to open port {spec.port}")

    try:
        if not port_handler.setBaudRate(spec.baudrate):
            return _make_transport_error_report(
                spec,
                f"failed to set baudrate {spec.baudrate}",
            )
        sample_count = max(1, int(samples))
        print(
            f"[{spec.role}] Hardware Error Status(70) check on IDs "
            f"{selected_motor_ids} ({sample_count} sample(s)):"
        )
        last_statuses: tuple[DynamixelMotorStatus, ...] = ()
        for sample_idx in range(sample_count):
            if sample_count > 1:
                print(f"  sample {sample_idx + 1}/{sample_count}:")
            try:
                statuses = _read_hardware_error_statuses(
                    packet_handler,
                    port_handler,
                    selected_motor_ids,
                )
            except Exception as exc:
                return _make_transport_error_report(
                    spec,
                    f"{type(exc).__name__}: {exc}",
                )
            last_statuses = statuses
            if statuses and all(status.ok for status in statuses):
                return DynamixelRoleStatusReport(
                    role=spec.role,
                    port=spec.port,
                    baudrate=spec.baudrate,
                    statuses=statuses,
                )
            if sample_idx + 1 < sample_count and sample_interval_sec > 0:
                time.sleep(sample_interval_sec)
        return DynamixelRoleStatusReport(
            role=spec.role,
            port=spec.port,
            baudrate=spec.baudrate,
            statuses=last_statuses,
        )
    finally:
        port_handler.closePort()


def print_dynamixel_report_errors(report: DynamixelRoleStatusReport, *, prefix: str = "ERROR:") -> None:
    for summary in report.problem_summaries():
        print(f"{prefix} {summary}", file=sys.stderr)


def _reboot_dynamixel_role(spec, *, motor_ids: Sequence[int] | None = None) -> bool:
    try:
        import dynamixel_sdk
    except ModuleNotFoundError:
        print(
            "ERROR: dynamixel_sdk is missing. Run this from the control conda env.",
            file=sys.stderr,
        )
        return False

    selected_motor_ids = _normalize_motor_ids(spec, motor_ids)
    print(f"[{spec.role}] rebooting Dynamixel IDs {selected_motor_ids}")
    port_handler = dynamixel_sdk.PortHandler(spec.port)
    packet_handler = dynamixel_sdk.PacketHandler(2.0)
    packet_handler.dxl = dynamixel_sdk

    if not port_handler.openPort():
        print(f"[{spec.role}] ERROR: failed to open port {spec.port}", file=sys.stderr)
        return False

    try:
        if not port_handler.setBaudRate(spec.baudrate):
            print(
                f"[{spec.role}] ERROR: failed to set baudrate {spec.baudrate}",
                file=sys.stderr,
            )
            return False

        print(f"[{spec.role}] hardware status before reboot:")
        _read_hardware_error_statuses(packet_handler, port_handler, selected_motor_ids)

        print(f"[{spec.role}] disabling torque")
        for motor_id in selected_motor_ids:
            comm_result, dxl_error = packet_handler.write1ByteTxRx(
                port_handler,
                int(motor_id),
                ADDR_TORQUE_ENABLE,
                0,
            )
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={int(motor_id):02d} torque_disable={packet_status}")

        time.sleep(0.2)

        print(f"[{spec.role}] rebooting motors")
        for motor_id in selected_motor_ids:
            comm_result, dxl_error = packet_handler.reboot(port_handler, int(motor_id))
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={int(motor_id):02d} reboot={packet_status}")
            time.sleep(0.25)

        time.sleep(DYNAMIXEL_REBOOT_SETTLE_SEC)

        print(f"[{spec.role}] hardware status after reboot:")
        after_statuses = _read_hardware_error_statuses(
            packet_handler,
            port_handler,
            selected_motor_ids,
        )
        failed_statuses = [status for status in after_statuses if not status.ok]
        if failed_statuses:
            print(
                f"[{spec.role}] ERROR: Dynamixel reboot did not clear all selected motors.",
                file=sys.stderr,
            )
            for status in failed_statuses:
                print(f"[{spec.role}] ERROR: {status.summary()}", file=sys.stderr)
            return False

        print(f"[{spec.role}] Dynamixel reboot verified clean.")
        return True
    finally:
        port_handler.closePort()


def _reboot_dynamixel_roles(
    specs,
    *,
    motor_ids_by_role: Mapping[str, Sequence[int]] | None = None,
) -> bool:
    ok = True
    selected_any = False
    for spec in specs:
        motor_type = _role_motor_type(spec)
        if motor_type != "dynamixel":
            print(f"[{spec.role}] skipping reboot for motor_type={motor_type}")
            continue
        selected_any = True
        role_motor_ids = None if motor_ids_by_role is None else motor_ids_by_role.get(spec.role)
        if not _reboot_dynamixel_role(spec, motor_ids=role_motor_ids):
            ok = False

    if not selected_any:
        print("No Dynamixel roles selected for reboot.")
    return ok


def preflight_dynamixel_role(spec) -> bool:
    motor_type = _role_motor_type(spec)
    if motor_type != "dynamixel":
        print(f"[{spec.role}] skipping Dynamixel preflight for motor_type={motor_type}")
        return True

    report = check_dynamixel_role_status(spec)
    if report.ok:
        print(f"[{spec.role}] Dynamixel status preflight clean.")
        return True

    print(
        f"[{spec.role}] Dynamixel status preflight found non-clean or non-responding motor IDs.",
        file=sys.stderr,
    )
    print_dynamixel_report_errors(report)
    if not report.can_attempt_reboot:
        print(
            f"[{spec.role}] ERROR: Cannot run reboot because the transport is not usable.",
            file=sys.stderr,
        )
        return False

    bad_ids = report.bad_motor_ids
    print(
        f"[{spec.role}] Running automatic Dynamixel reboot recovery for IDs {bad_ids}.",
        file=sys.stderr,
    )
    if not _reboot_dynamixel_role(spec, motor_ids=bad_ids):
        print(
            f"[{spec.role}] ERROR: Automatic Dynamixel reboot recovery failed.",
            file=sys.stderr,
        )
        return False
    return True


def preflight_dynamixel_roles(specs) -> bool:
    ok = True
    selected_any = False
    for spec in specs:
        if _role_motor_type(spec) == "dynamixel":
            selected_any = True
        if not preflight_dynamixel_role(spec):
            ok = False
    if not selected_any:
        print("No Dynamixel roles selected for preflight.")
    return ok
