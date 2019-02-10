package jsonrpc.client.generator;

/**
 * Date: 24.08.14
 * Time: 18:12
 * Strategy for generation request identificators
 *
 * @author Artem Prigoda
 */
public interface IdGenerator<T> {

    T generate();
}
