pub trait Loggable: Send + Sync {
    /// **ログのヘッダを取得**
    fn header(&self) -> String;

    /// **ログの出力**
    fn output_log(&self) -> String;
}
