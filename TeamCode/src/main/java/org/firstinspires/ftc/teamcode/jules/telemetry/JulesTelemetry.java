package org.firstinspires.ftc.teamcode.jules.telemetry;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * Telemetry proxy that mirrors outgoing data into the {@link JulesDataOrganizer}.
 */
public class JulesTelemetry implements Telemetry {

    private final Telemetry delegate;
    private final JulesDataOrganizer organizer;

    public JulesTelemetry(Telemetry delegate, JulesDataOrganizer organizer) {
        this.delegate = delegate;
        this.organizer = organizer;
    }

    public Telemetry getFtcTelemetry() {
        return this;
    }

    // ---------------------------------------------------------------------
    // Core addData mirroring
    // ---------------------------------------------------------------------

    @Override
    public Item addData(String caption, String format, Object... args) {
        mirrorFormatted(caption, format, args);
        return wrap(delegate.addData(caption, format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        mirrorValue(caption, value);
        return wrap(delegate.addData(caption, value));
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        mirrorFunc(caption, valueProducer, null);
        return wrap(delegate.addData(caption, valueProducer));
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        mirrorFunc(caption, valueProducer, format);
        return wrap(delegate.addData(caption, format, valueProducer));
    }

    @Override
    public boolean removeItem(Item item) {
        if (item != null) {
            organizer.removeTelemetryKey(item.getCaption());
        }
        return delegate.removeItem(unwrap(item));
    }

    @Override
    public void clear() {
        delegate.clear();
    }

    @Override
    public void clearAll() {
        organizer.clearTelemetry();
        delegate.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        return delegate.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return delegate.removeAction(token);
    }

    @Override
    public void speak(String text) {
        delegate.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        delegate.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        return delegate.update();
    }

    @Override
    public Line addLine() {
        return wrap(delegate.addLine());
    }

    @Override
    public Line addLine(String lineCaption) {
        return wrap(delegate.addLine(lineCaption));
    }

    @Override
    public boolean removeLine(Line line) {
        return delegate.removeLine(unwrap(line));
    }

    @Override
    public boolean isAutoClear() {
        return delegate.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        delegate.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return delegate.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        delegate.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return delegate.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        delegate.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return delegate.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        delegate.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        delegate.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return delegate.log();
    }

    private void mirrorValue(String caption, Object value) {
        if (caption == null) {
            return;
        }
        organizer.recordTelemetry(caption, value);
    }

    private void mirrorFormatted(String caption, String format, Object... args) {
        if (caption == null) {
            return;
        }
        organizer.recordTelemetry(caption, safeFormat(format, args));
    }

    private <T> void mirrorFunc(String caption, Func<T> func, String format) {
        if (caption == null) {
            return;
        }
        try {
            T value = func == null ? null : func.value();
            if (format == null) {
                organizer.recordTelemetry(caption, value);
            } else {
                organizer.recordTelemetry(caption, safeFormat(format, value));
            }
        } catch (Exception ignored) {
            // Swallow; telemetry shouldn't break if value producer throws
        }
    }

    private String safeFormat(String format, Object... args) {
        if (format == null) {
            return "";
        }
        try {
            return String.format(Locale.US, format, args);
        } catch (Exception ex) {
            return String.valueOf(format);
        }
    }

    private Item wrap(Item item) {
        return item == null ? null : new MirroredItem(item);
    }

    private Line wrap(Line line) {
        return line == null ? null : new MirroredLine(line);
    }

    private Item unwrap(Item item) {
        return item instanceof MirroredItem ? ((MirroredItem) item).delegate : item;
    }

    private Line unwrap(Line line) {
        return line instanceof MirroredLine ? ((MirroredLine) line).delegate : line;
    }

    private class MirroredLine implements Line {
        private final Line delegate;

        MirroredLine(Line delegate) {
            this.delegate = delegate;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            mirrorFormatted(caption, format, args);
            return wrap(delegate.addData(caption, format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            mirrorValue(caption, value);
            return wrap(delegate.addData(caption, value));
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            mirrorFunc(caption, valueProducer, null);
            return wrap(delegate.addData(caption, valueProducer));
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            mirrorFunc(caption, valueProducer, format);
            return wrap(delegate.addData(caption, format, valueProducer));
        }
    }

    private class MirroredItem implements Item {
        private final Item delegate;

        MirroredItem(Item delegate) {
            this.delegate = delegate;
        }

        @Override
        public String getCaption() {
            return delegate.getCaption();
        }

        @Override
        public Item setCaption(String caption) {
            String old = delegate.getCaption();
            organizer.removeTelemetryKey(old);
            delegate.setCaption(caption);
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            mirrorFormatted(delegate.getCaption(), format, args);
            delegate.setValue(format, args);
            return this;
        }

        @Override
        public Item setValue(Object value) {
            mirrorValue(delegate.getCaption(), value);
            delegate.setValue(value);
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            mirrorFunc(delegate.getCaption(), valueProducer, null);
            delegate.setValue(valueProducer);
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            mirrorFunc(delegate.getCaption(), valueProducer, format);
            delegate.setValue(format, valueProducer);
            return this;
        }

        @Override
        public Item setRetained(Boolean retained) {
            delegate.setRetained(retained);
            return this;
        }

        @Override
        public boolean isRetained() {
            return delegate.isRetained();
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            mirrorFormatted(caption, format, args);
            return wrap(delegate.addData(caption, format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            mirrorValue(caption, value);
            return wrap(delegate.addData(caption, value));
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            mirrorFunc(caption, valueProducer, null);
            return wrap(delegate.addData(caption, valueProducer));
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            mirrorFunc(caption, valueProducer, format);
            return wrap(delegate.addData(caption, format, valueProducer));
        }
    }
}

