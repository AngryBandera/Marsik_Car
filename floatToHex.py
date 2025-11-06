#!/usr/bin/env python3
"""
Telegram-бот для конвертації коефіцієнтів ПІД-регулятора у 6-байтний HEX-пакет.

Приймає команди типу: /pid kp 1200.0
"""

import struct
import logging
from telegram import Update
from telegram.ext import Application, CommandHandler, ContextTypes

# Configuration variables - replace with your values
BOT_TOKEN = '6667196781:AAGQuvq0FzLDW-DXVfkGytzFUm6sb2EIlF0'  # Replace with your actual bot token

# === Логіка з вашого 'ble_packer.py' ===

# Словник, що пов'язує назви коефіцієнтів з їхніми командами
COMMAND_MAP = {
    'kp': 0x10,
    'ki': 0x11,
    'kd': 0x12,
}

# Префікс, який наказує CM0+ переслати пакет на CM4
BLE_PREFIX = 0x02

def create_ble_packet(coeff_name: str, value: float) -> bytes:
    """
    Створює 6-байтний пакет.
    """
    command_byte = COMMAND_MAP.get(coeff_name.lower())
    if command_byte is None:
        raise ValueError(f"Невідомий коефіцієнт: {coeff_name}. Дозволено: kp, ki, kd.")

    packet = bytearray(6)
    packet[0] = BLE_PREFIX
    packet[1] = command_byte
    
    try:
        struct.pack_into('<f', packet, 2, value)
    except OverflowError:
        raise ValueError(f"Значення {value} завелике для float.")
        
    return packet

# === Кінець логіки з 'ble_packer.py' ===


# Налаштування логування, щоб бачити, що бот працює
logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=logging.INFO
)
logger = logging.getLogger(__name__)


async def start_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Надсилає інструкцію при командах /start або /help."""
    user = update.effective_user
    help_text = (
        f"Привіт, {user.first_name}!\n\n"
        "Я бот для ПІД-пакувальника. Я конвертую ваші коефіцієнти у HEX-пакет для BLE.\n\n"
        "<b>Використовуйте формат:</b>\n"
        "<code>/pid &lt;coeff&gt; &lt;value&gt;</code>\n\n"
        "<b>Доступні коефіцієнти:</b> <code>kp</code>, <code>ki</code>, <code>kd</code>\n\n"
        "<b>Приклад:</b>\n"
        "<code>/pid kp 1200.0</code>"
    )
    await update.message.reply_html(help_text)


async def pid_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Обробляє команду /pid."""
    
    # 1. Перевірка кількості аргументів
    if len(context.args) != 2:
        await update.message.reply_text(
            "Помилка: Неправильний формат.\n"
            "Потрібно 2 аргументи: /pid <coeff> <value>\n"
            "Приклад: /pid kp 1200.0"
        )
        return

    coeff_name = context.args[0]
    value_str = context.args[1]

    # 2. Перевірка типу значення
    try:
        value_float = float(value_str)
    except ValueError:
        await update.message.reply_text(
            f"Помилка: Значення '{value_str}' не є числом."
        )
        return

    # 3. Спроба створити пакет
    try:
        ble_packet = create_ble_packet(coeff_name, value_float)
        
        # 4. Надсилання успішного результату
        response_text = (
            f"<b>Коефіцієнт:</b> {coeff_name.upper()}\n"
            f"<b>Значення:</b>     {value_float}\n"
            "----------------------------\n"
            "✅ <b>Готовий HEX-пакет (6 байт):</b>\n"
            f"<code>{ble_packet.hex()}</code>"
        )
        await update.message.reply_html(response_text)

    except ValueError as e:
        # 4. Обробка помилок (неправильний коеф. або значення)
        await update.message.reply_text(f"Помилка: {e}")
    except Exception as e:
        logger.error(f"Невідома помилка: {e}")
        await update.message.reply_text(f"Виникла невідома помилка: {e}")


def main():
    """Запускає бота."""
    logger.info("Запуск бота...")
    
    # Створення Application
    application = Application.builder().token(BOT_TOKEN).build()

    # Додавання обробників команд
    application.add_handler(CommandHandler("start", start_command))
    application.add_handler(CommandHandler("help", start_command))
    application.add_handler(CommandHandler("pid", pid_command))

    # Запуск бота (очікування повідомлень)
    application.run_polling()


if __name__ == "__main__":
    main()