"""Native radio inputs bridged to Viser's button-group messages.

Viser 1.1.1 has no radio widget. Keep transport and client ownership in Viser;
only the radio presentation uses HTML. This adapter is tied to the pinned version.
"""

from html import escape


class ModeRadio:
    def __init__(self, gui, options, events):
        self.options = options
        self._value = options[0]
        self.bridge = gui.add_button_group("Mode transport", options)
        self.uuid = self.bridge._impl.uuid
        self.html = gui.add_html(self.render())

        @self.bridge.on_click
        async def selected(event):
            if event.client is not None:
                events.put(("mode", event.client.client_id, self.bridge.value))

    def render(self):
        selector = f'label[for="{self.uuid}"]'
        click = (
            f"document.querySelector('{selector}').closest('.mantine-Flex-root').querySelectorAll('button')"
        )
        rows = "".join(
            f'<label style="display:flex;align-items:center;gap:10px;padding:5px 0;cursor:pointer">'
            f'<input type="radio" name="so101-mode" value="{escape(option)}" '
            f"{'checked' if option == self._value else ''} "
            f'style="accent-color:#2dc3de" onchange="{escape(click, quote=True)}[{i}].click()">'
            f"{escape(option)}</label>"
            for i, option in enumerate(self.options)
        )
        return f'<style>.mantine-Flex-root:has({selector}){{display:none}}</style><fieldset style="border:0;padding:0 12px 10px;margin:0"><legend style="font-weight:600;margin-bottom:4px">Control mode</legend>{rows}</fieldset>'

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value
        self.html.content = self.render()
