use std::collections::HashMap;
use std::fmt::Display;

pub trait Loggable: Send + Sync {
    /// **ログのヘッダを取得**
    fn header(&self) -> String;

    /// **ログの出力**
    fn output_log(&self) -> String;
}

/// **Vec<T> に対する Loggable 実装**
impl<T: Loggable> Loggable for Vec<T> {
    fn header(&self) -> String {
        self.iter()
            .map(|item| item.header())
            .collect::<Vec<String>>()
            .join(", ")
    }

    fn output_log(&self) -> String {
        self.iter()
            .map(|item| item.output_log())
            .collect::<Vec<String>>()
            .join("\n")
    }
}

/// **HashMap<K, T> に対する Loggable 実装**
impl<K, T> Loggable for HashMap<K, T>
where
    K: Display + Send + Sync,
    T: Loggable,
{
    fn header(&self) -> String {
        self.iter()
            .map(|(key, value)| format!("{}: {}", key, value.header()))
            .collect::<Vec<String>>()
            .join(", ")
    }

    fn output_log(&self) -> String {
        self.iter()
            .map(|(key, value)| format!("{}: {}", key, value.output_log()))
            .collect::<Vec<String>>()
            .join("\n")
    }
}
