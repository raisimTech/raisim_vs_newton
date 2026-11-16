"""Shared validation and reporting for the standalone benchmark drivers."""
import math


def positive_price(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("Price must be a positive finite USD amount")
    return value


def prompt_prices(cpu=None, gpu=None, input_fn=input, output_fn=print):
    values = []
    for label, supplied in (("CPU", cpu), ("GPU", gpu)):
        if supplied is not None:
            values.append(positive_price(supplied))
            continue
        while True:
            try:
                values.append(positive_price(input_fn(f"How much did your {label} cost (USD)? ")))
                break
            except ValueError:
                output_fn("Enter a positive finite number, such as 339.99.")
            except EOFError as error:
                raise ValueError("Supply --cpu-price and --gpu-price for unattended runs") from error
    return tuple(values)
